#!/usr/bin/env python3
"""
Comprehensive RViz2 Visualization & Marker Publisher
Publishes:
- /defects (MarkerArray): Ground truth defects with geometry by type (crack=cube, rust=cylinder, bio=sphere),
  colored by state (UNDETECTED=grey, DETECTED=yellow, QUEUED=orange, CLEANING=pulsing red, VERIFIED_CLEAN=green),
  size proportional to severity, and text labels.
- /belief_map/heatmap (MarkerArray): 2D grid heatmap showing per-cell belief and utility.
- /task_queue_marker (Marker): HUD text display of utility-ranked tasks with current target highlighted.
- /mapper_1/sensor_fov (Marker): Transparent sensor footprint cone attached to mapper robot.
- /{robot_id}/path_marker (Marker): LINE_STRIP trajectory trail history.
- /{robot_id}/goal_marker (Marker): ARROW indicating active target goal.
- /visualization_marker_array (MarkerArray): Composite for backward compatibility.
"""

import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool

from hull_msgs.msg import DefectDetection, BeliefUpdate, TaskQueue


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')

        # Ground truth defect definitions (all start UNDETECTED)
        self.defects = {
            'defect_crack_01': {'type': 'crack', 'x': 4.0, 'y': 2.0, 'z': 0.05, 'sev': 0.95, 'state': 'UNDETECTED'},
            'defect_crack_02': {'type': 'crack', 'x': 8.5, 'y': 4.5, 'z': 0.05, 'sev': 0.88, 'state': 'UNDETECTED'},
            'defect_crack_03': {'type': 'crack', 'x': 14.0, 'y': 1.5, 'z': 0.05, 'sev': 0.80, 'state': 'UNDETECTED'},
            'defect_rust_01': {'type': 'rust', 'x': 3.0, 'y': 4.0, 'z': 0.05, 'sev': 0.75, 'state': 'UNDETECTED'},
            'defect_rust_02': {'type': 'rust', 'x': 11.0, 'y': 3.0, 'z': 0.05, 'sev': 0.70, 'state': 'UNDETECTED'},
            'defect_rust_03': {'type': 'rust', 'x': 16.5, 'y': 4.8, 'z': 0.05, 'sev': 0.65, 'state': 'UNDETECTED'},
            'defect_bio_01': {'type': 'biofouling', 'x': 6.0, 'y': 1.2, 'z': 0.05, 'sev': 0.60, 'state': 'UNDETECTED'},
            'defect_bio_02': {'type': 'biofouling', 'x': 12.5, 'y': 5.0, 'z': 0.05, 'sev': 0.55, 'state': 'UNDETECTED'},
            'defect_bio_03': {'type': 'biofouling', 'x': 17.5, 'y': 2.2, 'z': 0.05, 'sev': 0.50, 'state': 'UNDETECTED'},
        }

        # Robot poses, paths, and goals
        self.robot_poses = {'mapper_1': (1.5, 1.5, 0.0), 'cleaner_1': (1.5, 4.5, 0.0)}
        self.robot_paths = {'mapper_1': deque(maxlen=400), 'cleaner_1': deque(maxlen=400)}
        self.robot_goals = {'mapper_1': (2.0, 1.5), 'cleaner_1': None}
        self.cleaner_tool_active = False

        # Task queue storage
        self.ranked_tasks = []

        # Subscriptions
        self.create_subscription(PoseStamped, '/mapper_1/pose', lambda m: self._pose_cb('mapper_1', m), 10)
        self.create_subscription(PoseStamped, '/cleaner_1/pose', lambda m: self._pose_cb('cleaner_1', m), 10)
        self.create_subscription(PoseStamped, '/cleaner_1/target_goal', self._goal_cb, 10)
        self.create_subscription(Bool, '/cleaner_1/tool_cmd', self._tool_cb, 10)

        self.create_subscription(DefectDetection, '/defect_detections', self._detect_cb, 20)
        self.create_subscription(BeliefUpdate, '/belief_updates', self._belief_cb, 20)
        self.create_subscription(TaskQueue, '/task_queue', self._queue_cb, 10)

        # Publishers for all required visualization topics
        self.defects_pub = self.create_publisher(MarkerArray, '/defects', 10)
        self.heatmap_pub = self.create_publisher(MarkerArray, '/belief_map/heatmap', 10)
        self.queue_marker_pub = self.create_publisher(Marker, '/task_queue_marker', 10)
        self.fov_pub = self.create_publisher(Marker, '/mapper_1/sensor_fov', 10)
        self.mapper_path_pub = self.create_publisher(Marker, '/mapper_1/path_marker', 10)
        self.cleaner_path_pub = self.create_publisher(Marker, '/cleaner_1/path_marker', 10)
        self.cleaner_goal_pub = self.create_publisher(Marker, '/cleaner_1/goal_marker', 10)

        # Backward compatibility composite topic
        self.composite_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Broadcast timer (10 Hz)
        self.timer = self.create_timer(0.1, self._publish_all)
        self.get_logger().info('MarkerPublisher initialized with full defect, heatmap, FOV, trail, and HUD topics')

    def _pose_cb(self, rid, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_poses[rid] = (x, y, 0.05)
        # Append to trail if moved significantly
        path = self.robot_paths[rid]
        if not path or math.hypot(x - path[-1][0], y - path[-1][1]) > 0.08:
            path.append((x, y, 0.03))

    def _goal_cb(self, msg):
        self.robot_goals['cleaner_1'] = (msg.pose.position.x, msg.pose.position.y)

    def _tool_cb(self, msg):
        self.cleaner_tool_active = msg.data

    def _detect_cb(self, msg):
        d_id = msg.defect_id
        if d_id in self.defects:
            if self.defects[d_id]['state'] == 'UNDETECTED':
                self.defects[d_id]['state'] = 'DETECTED'
            self.defects[d_id]['sev'] = msg.severity

    def _belief_cb(self, msg):
        d_id = msg.defect_id
        if d_id in self.defects:
            self.defects[d_id]['state'] = msg.status
            self.defects[d_id]['sev'] = msg.severity
            if msg.status in ('VERIFIED_CLEAN', 'DONE'):
                self.defects[d_id]['state'] = 'VERIFIED_CLEAN'
                self.defects[d_id]['sev'] = 0.0

    def _queue_cb(self, msg):
        self.ranked_tasks = msg.tasks
        for task in msg.tasks:
            d_id = task.defect_id
            if d_id in self.defects:
                curr_state = self.defects[d_id]['state']
                if curr_state not in ('CLEANING', 'VERIFIED_CLEAN', 'DONE'):
                    self.defects[d_id]['state'] = 'QUEUED'

    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        t_sec = time.time()

        # 1. Publish /defects (MarkerArray)
        defects_array = MarkerArray()
        m_id = 0

        for d_id, d in self.defects.items():
            state = d['state']
            dtype = d['type']
            sev = d['sev']
            x, y, z = d['x'], d['y'], d['z']

            # Geometry Marker
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'hull_world'
            m.ns = 'defects_geometry'
            m.id = m_id
            m_id += 1
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0

            # Shape by type
            if dtype == 'crack':
                m.type = Marker.CUBE
                base_scale = 0.20 + 0.35 * sev
                m.scale.x = 0.12
                m.scale.y = base_scale * 1.5
                m.scale.z = 0.05
            elif dtype == 'rust':
                m.type = Marker.CYLINDER
                base_scale = 0.20 + 0.30 * sev
                m.scale.x = base_scale * 1.8
                m.scale.y = base_scale * 1.8
                m.scale.z = 0.04
            else:  # biofouling
                m.type = Marker.SPHERE
                base_scale = 0.22 + 0.32 * sev
                m.scale.x = base_scale * 1.8
                m.scale.y = base_scale * 1.8
                m.scale.z = base_scale * 0.8

            # Color by state:
            # UNDETECTED (grey, faint), DETECTED (yellow), QUEUED (orange), CLEANING (red, pulsing), VERIFIED_CLEAN (green)
            if state == 'UNDETECTED':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.50, 0.52, 0.58, 0.35
                m.scale.x *= 0.6
                m.scale.y *= 0.6
            elif state == 'DETECTED':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.90, 0.10, 0.95
            elif state == 'QUEUED':
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.55, 0.05, 0.95
            elif state == 'CLEANING':
                pulse = 1.0 + 0.18 * math.sin(t_sec * 8.0)
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.12, 0.12, 1.0
                m.scale.x *= pulse
                m.scale.y *= pulse
            elif state in ('VERIFIED_CLEAN', 'DONE'):
                m.color.r, m.color.g, m.color.b, m.color.a = 0.08, 0.95, 0.25, 0.95
                m.scale.x *= 0.85
                m.scale.y *= 0.85
            else:  # RE_QUEUED
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.40, 0.0, 1.0

            defects_array.markers.append(m)

            # Progress bar for CLEANING state
            if state == 'CLEANING':
                p_bar = Marker()
                p_bar.header.stamp = now
                p_bar.header.frame_id = 'hull_world'
                p_bar.ns = 'defects_progressbar'
                p_bar.id = m_id
                m_id += 1
                p_bar.type = Marker.CUBE
                p_bar.action = Marker.ADD
                p_bar.pose.position.x = x
                p_bar.pose.position.y = y + 0.35
                p_bar.pose.position.z = z + 0.08
                p_bar.pose.orientation.w = 1.0
                p_bar.scale.x = 0.50 * max(0.1, sev)
                p_bar.scale.y = 0.08
                p_bar.scale.z = 0.02
                p_bar.color.r, p_bar.color.g, p_bar.color.b, p_bar.color.a = 0.0, 1.0, 0.3, 0.9
                defects_array.markers.append(p_bar)

            # Text Label Marker
            txt = Marker()
            txt.header.stamp = now
            txt.header.frame_id = 'hull_world'
            txt.ns = 'defects_text'
            txt.id = m_id
            m_id += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = x
            txt.pose.position.y = y + 0.45
            txt.pose.position.z = 0.20
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.24
            if state == 'UNDETECTED':
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.6, 0.6, 0.6, 0.45
            elif state == 'VERIFIED_CLEAN':
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.3, 1.0, 0.5, 0.95
            elif state == 'CLEANING':
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 0.3, 0.3, 1.0
            else:
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 1.0, 1.0, 1.0
            txt.text = f"{d_id}\n[{dtype.upper()}] | {state}\nSev: {sev:.2f}"
            defects_array.markers.append(txt)

        self.defects_pub.publish(defects_array)

        # 2. Publish /belief_map/heatmap (MarkerArray)
        heatmap_array = MarkerArray()
        h_id = 0
        # 20m x 6m divided into 2m x 1m patches
        for gx in range(0, 20, 2):
            for gy in range(0, 6, 1):
                hm = Marker()
                hm.header.stamp = now
                hm.header.frame_id = 'hull_world'
                hm.ns = 'belief_heatmap'
                hm.id = h_id
                h_id += 1
                hm.type = Marker.CUBE
                hm.action = Marker.ADD
                hm.pose.position.x = gx + 1.0
                hm.pose.position.y = gy + 0.5
                hm.pose.position.z = 0.005
                hm.pose.orientation.w = 1.0
                hm.scale.x = 1.90
                hm.scale.y = 0.90
                hm.scale.z = 0.01

                # Calculate belief / utility density in this cell
                cell_cx, cell_cy = gx + 1.0, gy + 0.5
                cell_sev = 0.0
                cell_cleaned = False
                for d in self.defects.values():
                    if math.hypot(d['x'] - cell_cx, d['y'] - cell_cy) < 1.4:
                        if d['state'] in ('DETECTED', 'QUEUED', 'CLEANING'):
                            cell_sev += d['sev']
                        elif d['state'] == 'VERIFIED_CLEAN':
                            cell_cleaned = True

                if cell_sev > 0.4:
                    hm.color.r, hm.color.g, hm.color.b, hm.color.a = 0.90, 0.30, 0.10, 0.35  # High utility
                elif cell_cleaned:
                    hm.color.r, hm.color.g, hm.color.b, hm.color.a = 0.10, 0.85, 0.25, 0.25  # Cleaned zone
                else:
                    hm.color.r, hm.color.g, hm.color.b, hm.color.a = 0.15, 0.35, 0.65, 0.12  # Nominal zone

                heatmap_array.markers.append(hm)

        self.heatmap_pub.publish(heatmap_array)

        # 3. Publish /task_queue_marker (HUD text)
        hud = Marker()
        hud.header.stamp = now
        hud.header.frame_id = 'hull_world'
        hud.ns = 'task_queue_hud'
        hud.id = 5000
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD
        hud.pose.position.x = 10.0
        hud.pose.position.y = 6.8
        hud.pose.position.z = 0.5
        hud.pose.orientation.w = 1.0
        hud.scale.z = 0.32
        hud.color.r, hud.color.g, hud.color.b, hud.color.a = 1.0, 1.0, 1.0, 1.0

        hud_lines = ["=== COGNITIVE TASK QUEUE (UTILITY RANKED) ==="]
        if self.ranked_tasks:
            for idx, t in enumerate(self.ranked_tasks[:5]):
                d_info = self.defects.get(t.defect_id, {})
                status_str = d_info.get('state', t.status)
                if idx == 0 and status_str == 'CLEANING':
                    hud_lines.append(f">>> [ACTIVE TARGET] {t.defect_id} ({t.defect_type.upper()}) | U={t.utility_score:.2f} | STATE: CLEANING <<<")
                elif idx == 0:
                    hud_lines.append(f">>> [CURRENT GOAL]   {t.defect_id} ({t.defect_type.upper()}) | U={t.utility_score:.2f} | STATE: {status_str} <<<")
                else:
                    hud_lines.append(f"    [{idx+1}] QUEUED:     {t.defect_id} ({t.defect_type.upper()}) | U={t.utility_score:.2f} | STATE: {status_str}")
        else:
            hud_lines.append("    Scanning hull manifold... No active defects queued.")

        hud.text = "\n".join(hud_lines)
        self.queue_marker_pub.publish(hud)

        # 4. Publish /mapper_1/sensor_fov (Transparent Footprint Cone)
        mx, my, _ = self.robot_poses['mapper_1']
        fov = Marker()
        fov.header.stamp = now
        fov.header.frame_id = 'hull_world'
        fov.ns = 'sensor_fov'
        fov.id = 6000
        fov.type = Marker.CYLINDER
        fov.action = Marker.ADD
        fov.pose.position.x = mx
        fov.pose.position.y = my
        fov.pose.position.z = 0.03
        fov.pose.orientation.w = 1.0
        fov.scale.x = 3.6  # 1.8m radius -> 3.6m diameter
        fov.scale.y = 3.6
        fov.scale.z = 0.02
        fov.color.r, fov.color.g, fov.color.b, fov.color.a = 0.0, 0.85, 1.0, 0.22
        self.fov_pub.publish(fov)

        # 5. Publish Path Trails
        # Mapper 1 Trail
        m_trail = Marker()
        m_trail.header.stamp = now
        m_trail.header.frame_id = 'hull_world'
        m_trail.ns = 'mapper_path'
        m_trail.id = 7001
        m_trail.type = Marker.LINE_STRIP
        m_trail.action = Marker.ADD
        m_trail.scale.x = 0.06
        m_trail.color.r, m_trail.color.g, m_trail.color.b, m_trail.color.a = 0.0, 0.90, 1.0, 0.85
        m_trail.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.robot_paths['mapper_1']]
        self.mapper_path_pub.publish(m_trail)

        # Cleaner 1 Trail
        c_trail = Marker()
        c_trail.header.stamp = now
        c_trail.header.frame_id = 'hull_world'
        c_trail.ns = 'cleaner_path'
        c_trail.id = 7002
        c_trail.type = Marker.LINE_STRIP
        c_trail.action = Marker.ADD
        c_trail.scale.x = 0.08
        c_trail.color.r, c_trail.color.g, c_trail.color.b, c_trail.color.a = 1.0, 0.50, 0.0, 0.85
        c_trail.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.robot_paths['cleaner_1']]
        self.cleaner_path_pub.publish(c_trail)

        # 6. Publish /cleaner_1/goal_marker (Arrow)
        cx, cy, _ = self.robot_poses['cleaner_1']
        c_goal = self.robot_goals['cleaner_1']
        c_arrow = Marker()
        c_arrow.header.stamp = now
        c_arrow.header.frame_id = 'hull_world'
        c_arrow.ns = 'cleaner_goal'
        c_arrow.id = 8001
        c_arrow.type = Marker.ARROW
        c_arrow.action = Marker.ADD
        c_arrow.scale.x = 0.10  # shaft diameter
        c_arrow.scale.y = 0.22  # head diameter
        c_arrow.scale.z = 0.25  # head length
        c_arrow.color.r, c_arrow.color.g, c_arrow.color.b, c_arrow.color.a = 0.10, 1.0, 0.30, 0.90

        if c_goal:
            gx, gy = c_goal
            c_arrow.points = [
                Point(x=cx, y=cy, z=0.15),
                Point(x=gx, y=gy, z=0.15)
            ]
        else:
            c_arrow.points = [
                Point(x=cx, y=cy, z=0.15),
                Point(x=cx + 0.01, y=cy, z=0.15)
            ]
        self.cleaner_goal_pub.publish(c_arrow)

        # Composite array for backward compatibility
        composite = MarkerArray()
        composite.markers.extend(defects_array.markers)
        composite.markers.append(fov)
        composite.markers.append(hud)
        composite.markers.append(m_trail)
        composite.markers.append(c_trail)
        self.composite_pub.publish(composite)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MarkerPublisher()
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
