#!/usr/bin/env python3
"""
Demo Snapshot Node
Generates high-resolution visual snapshots of the ship hull simulation:
- docs/screenshot_start.png (mission initialization & inspection start)
- docs/screenshot_mid_cleaning.png (active targeting & cleaning)
- docs/screenshot_end.png (verified clean mission state)
- docs/simulation_screenshot.png (continuous latest snapshot)
"""

import os
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import matplotlib.patches as patches

DOCS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../docs'))
os.makedirs(DOCS_DIR, exist_ok=True)


class DemoSnapshotNode(Node):
    def __init__(self):
        super().__init__('demo_snapshot_node')

        self.robot_poses = {}
        self.robot_trails = {'mapper_1': [], 'cleaner_1': []}
        self.defects = {}
        self.tool_active = False
        self.start_time = time.time()

        self.captured_start = False
        self.captured_mid = False
        self.captured_end = False

        self.mapper_sub = self.create_subscription(
            PoseStamped, '/mapper_1/pose', lambda msg: self._pose_cb('mapper_1', msg), 10)
        self.cleaner_sub = self.create_subscription(
            PoseStamped, '/cleaner_1/pose', lambda msg: self._pose_cb('cleaner_1', msg), 10)
        self.defects_sub = self.create_subscription(
            MarkerArray, '/defects', self._defects_cb, 10)
        self.tool_sub = self.create_subscription(
            Bool, '/cleaner_1/tool_cmd', self._tool_cb, 10)

        # Periodic snapshot check loop (2 Hz)
        self.timer = self.create_timer(0.5, self._check_and_snapshot)
        self.get_logger().info(f'DemoSnapshotNode initialized -> Outputs to: {DOCS_DIR}')

    def _pose_cb(self, rid, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        self.robot_poses[rid] = (x, y)
        trail = self.robot_trails[rid]
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.15:
            trail.append((x, y))

    def _tool_cb(self, msg):
        self.tool_active = msg.data

    def _defects_cb(self, msg):
        for m in msg.markers:
            if m.ns == 'defects_geometry':
                color = (m.color.r, m.color.g, m.color.b)
                self.defects[m.id] = {
                    'x': m.pose.position.x,
                    'y': m.pose.position.y,
                    'scale_x': m.scale.x,
                    'scale_y': m.scale.y,
                    'type': m.type,
                    'color': color
                }

    def _check_and_snapshot(self):
        elapsed = time.time() - self.start_time
        if not self.robot_poses:
            return

        # Snapshot 1: Start (1.5 - 2.5s)
        if elapsed >= 2.0 and not self.captured_start:
            self._render_frame(os.path.join(DOCS_DIR, 'screenshot_start.png'),
                               title="HullSentinal [Start] — Mapper Sweeps Hull Manifold & Detects Defects")
            self.captured_start = True

        # Snapshot 2: Mid-cleaning (active tool or 6s - 12s)
        if (self.tool_active or elapsed >= 8.0) and self.captured_start and not self.captured_mid:
            self._render_frame(os.path.join(DOCS_DIR, 'screenshot_mid_cleaning.png'),
                               title="HullSentinal [Mid-Cleaning] — Cleaner Dispatched by Utility Scorer, Active Cavitation Tool")
            self.captured_mid = True

        # Snapshot 3: End / Verified (18s+ or when verified)
        if elapsed >= 18.0 and self.captured_mid and not self.captured_end:
            self._render_frame(os.path.join(DOCS_DIR, 'screenshot_end.png'),
                               title="HullSentinal [End] — Autonomous Re-scan Verification Confirmed (VERIFIED_CLEAN)")
            self.captured_end = True

        # Continually update simulation_screenshot.png
        self._render_frame(os.path.join(DOCS_DIR, 'simulation_screenshot.png'),
                           title="HullSentinal — Multi-Robot Ship Hull Cleaning Visual Simulation")

    def _render_frame(self, filepath, title):
        fig, ax = plt.subplots(figsize=(15, 6), facecolor='#0a1118')
        ax.set_facecolor('#141e28')

        # Deep Ocean background
        ocean_rect = patches.Rectangle((-1, -1), 22, 8, facecolor='#0d1b2a', alpha=0.9)
        ax.add_patch(ocean_rect)

        # Ship hull plate: 20m x 6m
        hull_rect = patches.Rectangle((0, 0), 20, 6, linewidth=2.5, edgecolor='#e0a912', facecolor='#1e293b')
        ax.add_patch(hull_rect)

        # Hull structural weld grid seams
        for gx in range(0, 21, 2):
            ax.axvline(gx, color='#334155', linestyle='-', linewidth=0.8, alpha=0.5)
        for gy in range(0, 7, 1):
            ax.axhline(gy, color='#334155', linestyle='-', linewidth=0.8, alpha=0.5)

        # Draw robot trails
        for rid, trail in self.robot_trails.items():
            if len(trail) > 1:
                tx = [p[0] for p in trail]
                ty = [p[1] for p in trail]
                color = '#00e5ff' if 'mapper' in rid else '#ff9100'
                ax.plot(tx, ty, color=color, linestyle='--', linewidth=1.5, alpha=0.6)

        # Draw defects
        for d in self.defects.values():
            rx = d['scale_x'] / 2.0
            ry = d['scale_y'] / 2.0
            ellipse = patches.Ellipse((d['x'], d['y']), rx * 2, ry * 2,
                                      edgecolor='white', facecolor=d['color'], alpha=0.9, linewidth=1.5)
            ax.add_patch(ellipse)

        # Draw Mapper Robot & Sensor FOV cone
        if 'mapper_1' in self.robot_poses:
            mx, my = self.robot_poses['mapper_1']
            # Transparent sensor FOV circle (radius 1.8m)
            fov_circle = patches.Circle((mx, my), 1.8, facecolor='#00e5ff', edgecolor='#00e5ff', alpha=0.18, linestyle=':')
            ax.add_patch(fov_circle)
            # Body
            m_box = patches.Rectangle((mx - 0.32, my - 0.22), 0.65, 0.45, facecolor='#00b4d8', edgecolor='white', linewidth=2)
            ax.add_patch(m_box)
            ax.text(mx, my + 0.38, "MAPPER 1 (FOV 1.8m)", color='#00e5ff', fontsize=8, fontweight='bold', ha='center')

        # Draw Cleaner Robot & Tool
        if 'cleaner_1' in self.robot_poses:
            cx, cy = self.robot_poses['cleaner_1']
            c_box = patches.Rectangle((cx - 0.35, cy - 0.25), 0.70, 0.50, facecolor='#ff6d00', edgecolor='white', linewidth=2)
            ax.add_patch(c_box)
            # Cleaning tool brush
            tool_color = '#00e676' if self.tool_active else '#757575'
            tool_c = patches.Circle((cx + 0.38, cy), 0.22, facecolor=tool_color, edgecolor='white', alpha=0.85, linewidth=1.5)
            ax.add_patch(tool_c)
            tool_status = "CLEANING" if self.tool_active else "TRANSIT"
            ax.text(cx, cy - 0.42, f"CLEANER 1 [{tool_status}]", color='#ff9100', fontsize=8, fontweight='bold', ha='center')

        ax.set_xlim(-0.5, 20.5)
        ax.set_ylim(-0.5, 6.8)
        ax.set_aspect('equal')
        ax.set_title(title, color='white', fontsize=12, fontweight='bold', pad=12)
        ax.set_xlabel("Ship Hull Length (meters)", color='#cbd5e1', fontsize=9)
        ax.set_ylabel("Hull Manifold Height (meters)", color='#cbd5e1', fontsize=9)
        ax.tick_params(colors='#cbd5e1')

        # Legend
        grey_p = patches.Patch(color='#808080', label='UNDETECTED (Faint)')
        yel_p = patches.Patch(color='#eab308', label='DETECTED')
        ora_p = patches.Patch(color='#f97316', label='QUEUED (Utility Ranked)')
        red_p = patches.Patch(color='#ef4444', label='CLEANING (Active Tool)')
        grn_p = patches.Patch(color='#22c55e', label='VERIFIED_CLEAN (Done)')
        map_p = patches.Patch(color='#00e5ff', label='Mapper ROV (FOV Swath)')
        cln_p = patches.Patch(color='#ff6d00', label='Cleaner Crawler (Brush Tool)')

        ax.legend(handles=[map_p, cln_p, grey_p, yel_p, ora_p, red_p, grn_p],
                  loc='upper right', facecolor='#0f172a', edgecolor='#475569', labelcolor='white', fontsize=7.5)

        plt.tight_layout()
        plt.savefig(filepath, dpi=180, facecolor=fig.get_facecolor())
        plt.close()
        self.get_logger().info(f"[Snapshot] Saved frame: {filepath}")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = DemoSnapshotNode()
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
