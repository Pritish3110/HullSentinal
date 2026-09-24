#!/usr/bin/env python3
"""
Belief Map Node
Maintains the probabilistic belief state of all hull defects using SQLAlchemy ORM.
Handles defect lifecycle:
UNDETECTED -> DETECTED -> QUEUED -> CLEANING -> VERIFIED_CLEAN (or RE_QUEUED)
"""

import time
import rclpy
from rclpy.node import Node
from sqlalchemy import create_engine, Column, String, Float, Integer
from sqlalchemy.orm import declarative_base, sessionmaker

from hull_msgs.msg import DefectDetection, BeliefUpdate, TaskQueue, TaskItem

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
    status = Column(String, default='DETECTED')
    rescan_count = Column(Integer, default=0)
    created_at = Column(Float)
    updated_at = Column(Float)


class BeliefMapNode(Node):
    def __init__(self):
        super().__init__('belief_map_node')

        self.declare_parameter('db_path', ':memory:')
        db_path = self.get_parameter('db_path').get_parameter_value().string_value

        # Initialize SQLite database via SQLAlchemy
        conn_str = 'sqlite://' if db_path == ':memory:' else f'sqlite:///{db_path}'
        self.engine = create_engine(conn_str, echo=False)
        Base.metadata.create_all(self.engine)
        self.Session = sessionmaker(bind=self.engine)

        # Fast in-memory dictionary cache for real-time ROS pub/sub loops
        self.cache = {}

        # Subscriptions
        self.detect_sub = self.create_subscription(
            DefectDetection, '/defect_detections', self._detection_cb, 20)
        self.update_sub = self.create_subscription(
            BeliefUpdate, '/belief_updates', self._update_cb, 20)
        self.queue_sub = self.create_subscription(
            TaskQueue, '/task_queue', self._queue_cb, 10)

        # Publishers
        self.update_pub = self.create_publisher(BeliefUpdate, '/belief_updates', 20)
        self.active_pub = self.create_publisher(TaskQueue, '/belief_map/active_defects', 10)

        # Periodic sync / publish timer (2 Hz)
        self.timer = self.create_timer(0.5, self._publish_active)
        self.get_logger().info(f'BeliefMapNode initialized with database: {db_path}')

    def _detection_cb(self, msg):
        """Ingests new defect detection from mapper inspection."""
        session = self.Session()
        try:
            d_id = msg.defect_id
            rec = session.query(DefectBeliefModel).filter_by(defect_id=d_id).first()
            now_ts = time.time()

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
                    status='DETECTED',
                    rescan_count=0,
                    created_at=now_ts,
                    updated_at=now_ts
                )
                session.add(rec)
                self.get_logger().info(f"[BeliefMap] New defect registered: {d_id} ({msg.defect_type}) -> DETECTED (sev={msg.severity:.2f})")

                # Broadcast belief update for DETECTED
                b_up = BeliefUpdate()
                b_up.defect_id = d_id
                b_up.status = 'DETECTED'
                b_up.severity = float(msg.severity)
                b_up.confidence = float(msg.confidence)
                b_up.rescan_count = 0
                b_up.stamp = self.get_clock().now().to_msg()
                self.update_pub.publish(b_up)
            else:
                # Bayesian / moving average update
                rec.severity = float(0.6 * rec.severity + 0.4 * msg.severity)
                rec.confidence = float(max(rec.confidence, msg.confidence))
                rec.updated_at = now_ts

            session.commit()
            self._update_cache_record(rec)
        finally:
            session.close()

    def _queue_cb(self, msg):
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
                    b_up.stamp = self.get_clock().now().to_msg()
                    self.update_pub.publish(b_up)
                    self.get_logger().info(f"[BeliefMap] Task {d_id} scored -> QUEUED")
        finally:
            session.close()

    def _update_cb(self, msg):
        """Applies status / verification updates (e.g. CLEANING, VERIFIED_CLEAN, RE_QUEUED)."""
        session = self.Session()
        try:
            rec = session.query(DefectBeliefModel).filter_by(defect_id=msg.defect_id).first()
            if rec:
                rec.status = msg.status
                rec.severity = float(msg.severity)
                rec.confidence = float(msg.confidence)
                rec.rescan_count = int(msg.rescan_count)
                rec.updated_at = time.time()
                session.commit()
                self._update_cache_record(rec)
                self.get_logger().info(f"[BeliefMap] Updated {msg.defect_id} -> status={msg.status} sev={msg.severity:.2f}")
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
            'status': rec.status,
            'rescan_count': rec.rescan_count
        }

    def _publish_active(self):
        """Publishes all active defects to the utility scorer."""
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
