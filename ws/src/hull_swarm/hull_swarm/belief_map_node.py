#!/usr/bin/env python3
"""
HullSentinal V2 — Belief Map Node
Centralized probabilistic world state engine maintaining full defect schema:
lane_id, risk_level, emergency_flag, eligible_lane, assigned_robot.
Handles defect lifecycle and temporal degradation tracking.
"""

import os
import time
import yaml
import rclpy
from rclpy.node import Node
from sqlalchemy import create_engine, Column, String, Float, Integer, Boolean
from sqlalchemy.orm import declarative_base, sessionmaker
from ament_index_python.packages import get_package_share_directory

from hull_msgs.msg import DefectDetection, BeliefUpdate, TaskQueue, TaskItem, LaneCoordination

Base = declarative_base()


class DefectBeliefModel(Base):
    __tablename__ = 'defect_beliefs'

    defect_id = Column(String, primary_key=True)
    defect_type = Column(String, nullable=False)
    severity = Column(Float, nullable=False)
    confidence = Column(Float, nullable=False)
    pos_x = Column(Float, nullable=False)
    pos_y = Column(Float, nullable=False)
    pos_z = Column(Float, default=0.0)
    lane_id = Column(Integer, default=1)
    risk_level = Column(String, default='NORMAL')
    emergency_flag = Column(Boolean, default=False)
    eligible_lane = Column(Boolean, default=False)
    assigned_robot = Column(String, default='')
    status = Column(String, default='DETECTED')
    rescan_count = Column(Integer, default=0)
    growth_rate = Column(Float, default=0.0)
    created_at = Column(Float)
    updated_at = Column(Float)


class BeliefMapNode(Node):
    def __init__(self):
        super().__init__('belief_map_node')

        self.declare_parameter('db_path', ':memory:')
        self.declare_parameter('config_file', '')
        db_path = self.get_parameter('db_path').get_parameter_value().string_value

        # Load master config
        self.cfg = self._load_config()
        self.defects_cfg = {d['id']: d for d in self.cfg.get('defects', [])}

        # Initialize SQLite database via SQLAlchemy
        conn_str = 'sqlite://' if db_path == ':memory:' else f'sqlite:///{db_path}'
        self.engine = create_engine(conn_str, echo=False)
        Base.metadata.create_all(self.engine)
        self.Session = sessionmaker(bind=self.engine)

        # Fast in-memory cache for high-rate pub/sub lookups
        self.cache = {}
        self.released_lane = 0  # 0: no lane released yet; cleaner waits for lane 1 release

        # Subscriptions
        self.detect_sub = self.create_subscription(
            DefectDetection, '/defect_detections', self._detection_cb, 20)
        self.update_sub = self.create_subscription(
            BeliefUpdate, '/belief_updates', self._update_cb, 20)
        self.queue_sub = self.create_subscription(
            TaskQueue, '/task_queue', self._queue_cb, 10)
        self.lane_coord_sub = self.create_subscription(
            LaneCoordination, '/lane_coordination/events', self._lane_coord_cb, 10)

        # Publishers
        self.update_pub = self.create_publisher(BeliefUpdate, '/belief_updates', 20)
        self.active_pub = self.create_publisher(TaskQueue, '/belief_map/active_defects', 10)

        # Periodic sync / publish timer (2 Hz)
        self.timer = self.create_timer(0.5, self._publish_active)
        self.last_sync_time = time.time()
        self.get_logger().info(f'[BELIEF] BeliefMapNode V2 initialized (DB: {db_path})')

    def _load_config(self):
        custom_path = self.get_parameter('config_file').get_parameter_value().string_value
        candidate_paths = [
            custom_path,
            os.path.join(get_package_share_directory('hull_bringup'), 'config', 'hullsentinal_v2.yaml')
            if 'hull_bringup' in os.environ.get('AMENT_PREFIX_PATH', '') else '',
            '/home/pritish_3110/Personal Files/Syllabus/Projects/HullSentinal/ws/src/hull_bringup/config/hullsentinal_v2.yaml',
        ]
        for path in candidate_paths:
            if path and os.path.exists(path):
                self.get_logger().info(f'[BELIEF] Loaded configuration from {path}')
                with open(path, 'r') as f:
                    return yaml.safe_load(f)
        return {}

    def _compute_risk_level(self, severity: float) -> str:
        if severity >= 0.85:
            return 'CRITICAL'
        elif severity >= 0.70:
            return 'HIGH'
        elif severity >= 0.50:
            return 'MEDIUM'
        return 'NORMAL'

    def _get_lane_id(self, d_id: str, y_pos: float) -> int:
        if d_id in self.defects_cfg:
            return int(self.defects_cfg[d_id].get('lane_id', 1))
        # Fallback geometric lane assignment for 3 lanes
        if y_pos < 3.0:
            return 1
        elif y_pos < 5.0:
            return 2
        return 3

    def _get_growth_rate(self, d_id: str) -> float:
        if d_id in self.defects_cfg:
            return float(self.defects_cfg[d_id].get('growth_rate', 0.0001))
        return 0.0001

    def _detection_cb(self, msg):
        """Processes incoming defect detections from mapper observation cone."""
        session = self.Session()
        try:
            d_id = msg.defect_id
            rec = session.query(DefectBeliefModel).filter_by(defect_id=d_id).first()
            now_ts = time.time()
            risk = self._compute_risk_level(msg.severity)
            is_emergency = (risk == 'CRITICAL')
            lane = self._get_lane_id(d_id, msg.position.y)
            growth = self._get_growth_rate(d_id)
            eligible = is_emergency or (lane <= self.released_lane)

            if not rec:
                # Transition UNDETECTED -> DETECTED
                rec = DefectBeliefModel(
                    defect_id=d_id,
                    defect_type=msg.defect_type,
                    severity=float(msg.severity),
                    confidence=float(msg.confidence),
                    pos_x=float(msg.position.x),
                    pos_y=float(msg.position.y),
                    pos_z=float(msg.position.z),
                    lane_id=lane,
                    risk_level=risk,
                    emergency_flag=is_emergency,
                    eligible_lane=eligible,
                    assigned_robot='',
                    status='DETECTED',
                    rescan_count=0,
                    growth_rate=growth,
                    created_at=now_ts,
                    updated_at=now_ts
                )
                session.add(rec)
                session.commit()
                self._update_cache_record(rec)

                self.get_logger().info(
                    f"[BELIEF] Registered {d_id} ({msg.defect_type}) at ({msg.position.x:.1f}, {msg.position.y:.1f}) "
                    f"in Lane {lane}: sev={msg.severity:.2f}, risk={risk}, emergency={is_emergency}, eligible={eligible}"
                )
                if is_emergency:
                    self.get_logger().warning(
                        f"[EMERGENCY] Detected defect {d_id} has CRITICAL risk ({msg.severity:.2f})! Immediate eligibility override."
                    )

                # Emit initial belief update
                b_up = BeliefUpdate()
                b_up.defect_id = d_id
                b_up.status = 'DETECTED'
                b_up.severity = float(msg.severity)
                b_up.confidence = float(msg.confidence)
                b_up.rescan_count = 0
                b_up.lane_id = lane
                b_up.risk_level = risk
                b_up.emergency_flag = is_emergency
                b_up.eligible_lane = eligible
                b_up.assigned_robot = ''
                b_up.stamp = self.get_clock().now().to_msg()
                self.update_pub.publish(b_up)
            else:
                # Bayesian / recursive filter update
                old_risk = rec.risk_level
                rec.severity = float(0.5 * rec.severity + 0.5 * msg.severity)
                rec.confidence = float(max(rec.confidence, msg.confidence))
                rec.risk_level = self._compute_risk_level(rec.severity)
                if rec.risk_level == 'CRITICAL':
                    rec.emergency_flag = True
                    rec.eligible_lane = True
                    if old_risk != 'CRITICAL':
                        self.get_logger().warning(
                            f"[EMERGENCY] Defect {d_id} in Lane {rec.lane_id} escalated to CRITICAL "
                            f"(sev={rec.severity:.2f}) via mapper re-observation! Triggering emergency override."
                        )
                rec.updated_at = now_ts
                session.commit()
                self._update_cache_record(rec)

                # Broadcast update if severity or risk changed
                b_up = BeliefUpdate()
                b_up.defect_id = d_id
                b_up.status = rec.status
                b_up.severity = rec.severity
                b_up.confidence = rec.confidence
                b_up.rescan_count = rec.rescan_count
                b_up.lane_id = rec.lane_id
                b_up.risk_level = rec.risk_level
                b_up.emergency_flag = rec.emergency_flag
                b_up.eligible_lane = rec.eligible_lane
                b_up.assigned_robot = rec.assigned_robot
                b_up.stamp = self.get_clock().now().to_msg()
                self.update_pub.publish(b_up)
        finally:
            session.close()

    def _lane_coord_cb(self, msg: LaneCoordination):
        """Receives lane coordination events from LaneCoordinatorNode."""
        self.released_lane = max(self.released_lane, msg.released_cleaner_lane)
        self.get_logger().info(
            f"[COORDINATION] BeliefMap updated released_cleaner_lane = {self.released_lane} "
            f"(event={msg.event})"
        )
        session = self.Session()
        try:
            records = session.query(DefectBeliefModel).all()
            for r in records:
                if r.emergency_flag or (r.lane_id <= self.released_lane):
                    if not r.eligible_lane:
                        r.eligible_lane = True
                        r.updated_at = time.time()
                        self._update_cache_record(r)
            session.commit()
        finally:
            session.close()

    def _queue_cb(self, msg: TaskQueue):
        """When tasks are ranked by utility scorer, transition DETECTED -> QUEUED."""
        session = self.Session()
        try:
            for task in msg.tasks:
                d_id = task.defect_id
                rec = session.query(DefectBeliefModel).filter_by(defect_id=d_id).first()
                if rec and rec.status == 'DETECTED':
                    rec.status = 'QUEUED'
                    rec.updated_at = time.time()
                    session.commit()
                    self._update_cache_record(rec)

                    b_up = BeliefUpdate()
                    b_up.defect_id = d_id
                    b_up.status = 'QUEUED'
                    b_up.severity = rec.severity
                    b_up.confidence = rec.confidence
                    b_up.rescan_count = rec.rescan_count
                    b_up.lane_id = rec.lane_id
                    b_up.risk_level = rec.risk_level
                    b_up.emergency_flag = rec.emergency_flag
                    b_up.eligible_lane = rec.eligible_lane
                    b_up.assigned_robot = rec.assigned_robot
                    b_up.stamp = self.get_clock().now().to_msg()
                    self.update_pub.publish(b_up)
        finally:
            session.close()

    def _update_cb(self, msg: BeliefUpdate):
        """Handles status / verification updates (CLEANING, VERIFIED_CLEAN, RE_QUEUED, etc.)."""
        session = self.Session()
        try:
            rec = session.query(DefectBeliefModel).filter_by(defect_id=msg.defect_id).first()
            if rec:
                rec.status = msg.status
                rec.severity = float(msg.severity)
                rec.confidence = float(msg.confidence)
                rec.rescan_count = int(msg.rescan_count)
                if msg.risk_level:
                    rec.risk_level = msg.risk_level
                else:
                    rec.risk_level = self._compute_risk_level(rec.severity)
                rec.emergency_flag = (rec.risk_level == 'CRITICAL')
                if rec.emergency_flag:
                    rec.eligible_lane = True
                if msg.assigned_robot:
                    rec.assigned_robot = msg.assigned_robot
                rec.updated_at = time.time()
                session.commit()
                self._update_cache_record(rec)
                self.get_logger().info(
                    f"[BELIEF] Updated {msg.defect_id}: status={msg.status}, sev={msg.severity:.2f}, "
                    f"risk={rec.risk_level}, eligible={rec.eligible_lane}"
                )
        finally:
            session.close()

    def _update_cache_record(self, rec):
        self.cache[rec.defect_id] = {
            'defect_id': rec.defect_id,
            'defect_type': rec.defect_type,
            'severity': rec.severity,
            'confidence': rec.confidence,
            'pos_x': rec.pos_x,
            'pos_y': rec.pos_y,
            'pos_z': rec.pos_z,
            'lane_id': rec.lane_id,
            'risk_level': rec.risk_level,
            'emergency_flag': rec.emergency_flag,
            'eligible_lane': rec.eligible_lane,
            'assigned_robot': rec.assigned_robot,
            'status': rec.status,
            'rescan_count': rec.rescan_count,
            'growth_rate': rec.growth_rate
        }

    def _publish_active(self):
        """Temporal projection and dispatch of active defect belief state."""
        now_ts = time.time()
        dt = max(0.01, now_ts - self.last_sync_time)
        self.last_sync_time = now_ts

        # Temporal defect growth projection on active defects
        session = self.Session()
        try:
            for d_id, d in list(self.cache.items()):
                if d['status'] in ('DETECTED', 'QUEUED', 'RE_QUEUED') and d['growth_rate'] > 0:
                    old_sev = d['severity']
                    new_sev = min(1.0, old_sev + d['growth_rate'] * dt)
                    d['severity'] = new_sev
                    if old_sev < 0.85 and new_sev >= 0.85:
                        d['risk_level'] = 'CRITICAL'
                        d['emergency_flag'] = True
                        d['eligible_lane'] = True
                        self.get_logger().warning(
                            f"[EMERGENCY] Cognitive map temporal projection: defect {d_id} in Lane {d['lane_id']} "
                            f"escalated past CRITICAL (sev={new_sev:.3f})! Emergency override active."
                        )
                        # Sync to DB
                        rec = session.query(DefectBeliefModel).filter_by(defect_id=d_id).first()
                        if rec:
                            rec.severity = new_sev
                            rec.risk_level = 'CRITICAL'
                            rec.emergency_flag = True
                            rec.eligible_lane = True
                            session.commit()

                        # Emit emergency belief update
                        b_up = BeliefUpdate()
                        b_up.defect_id = d_id
                        b_up.status = d['status']
                        b_up.severity = float(new_sev)
                        b_up.confidence = float(d['confidence'])
                        b_up.rescan_count = int(d['rescan_count'])
                        b_up.lane_id = int(d['lane_id'])
                        b_up.risk_level = 'CRITICAL'
                        b_up.emergency_flag = True
                        b_up.eligible_lane = True
                        b_up.assigned_robot = d['assigned_robot']
                        b_up.stamp = self.get_clock().now().to_msg()
                        self.update_pub.publish(b_up)
        finally:
            session.close()

        # Build TaskQueue for utility scorer
        msg = TaskQueue()
        msg.header.stamp = self.get_clock().now().to_msg()
        tasks = []

        for d in self.cache.values():
            if d['status'] in ('DETECTED', 'QUEUED', 'RE_QUEUED', 'PENDING', 'IN_PROGRESS'):
                item = TaskItem()
                item.defect_id = d['defect_id']
                item.defect_type = d['defect_type']
                item.utility_score = 0.0
                item.position.x = d['pos_x']
                item.position.y = d['pos_y']
                item.position.z = d['pos_z']
                item.status = d['status']
                item.lane_id = int(d['lane_id'])
                item.risk_level = d['risk_level']
                item.emergency_flag = bool(d['emergency_flag'])
                item.eligible = bool(d['eligible_lane'])
                item.path_cost = 0.0
                tasks.append(item)

        msg.tasks = tasks
        self.active_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = BeliefMapNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        except Exception:
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
