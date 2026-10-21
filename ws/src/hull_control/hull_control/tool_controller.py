#!/usr/bin/env python3
"""
HullSentinal V2 — Tool Controller Node
High-level cognitive cleaning executive implementing:
1. Explicit Enums & State Machine:
   (WAITING_FOR_LANE, IDLE, NAVIGATING, CLEANING, EMERGENCY_NAVIGATING, EMERGENCY_CLEANING, VERIFYING, COMPLETED).
2. Priority context stack for immediate CRITICAL defect emergency interruption & resume.
3. Waypoint-by-waypoint path following via A* grid planner.
4. Closed-loop verification handshake (/task_complete -> Rescan -> VERIFIED_CLEAN).
"""

import os
import math
import time
import yaml
import traceback
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

from hull_msgs.msg import TaskQueue, TaskItem, BeliefUpdate, LaneCoordination
from hull_msgs.srv import TaskComplete
from hull_control.grid_planner import GridPlanner


class CleanerState(Enum):
    WAITING_FOR_LANE = 'WAITING_FOR_LANE'
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    CLEANING = 'CLEANING'
    EMERGENCY_NAVIGATING = 'EMERGENCY_NAVIGATING'
    EMERGENCY_CLEANING = 'EMERGENCY_CLEANING'
    VERIFYING = 'VERIFYING'
    COMPLETED = 'COMPLETED'


class ToolController(Node):
    def __init__(self):
        super().__init__('tool_controller')

        self.declare_parameter('robot_id', 'cleaner_1')
        self.declare_parameter('config_file', '')
        self.declare_parameter('arrival_distance', 0.40)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.arrival_dist = self.get_parameter('arrival_distance').get_parameter_value().double_value

        # Load master config
        self.cfg = self._load_config()
        cleaner_cfg = self.cfg.get('robots', {}).get(self.robot_id, {})
        self.clean_duration = float(cleaner_cfg.get('clean_duration', 1.00))

        # Build A* Grid Planner
        world_cfg = self.cfg.get('world', {})
        hw = float(world_cfg.get('hull_width', 24.0))
        hh = float(world_cfg.get('hull_height', 8.0))
        margin = float(world_cfg.get('safety_margin', 0.8))
        obstacles = self.cfg.get('static_obstacles', [])
        self.grid_planner = GridPlanner(
            hull_width=hw,
            hull_height=hh,
            resolution=0.20,
            inflation_radius=0.55,
            safety_margin=margin,
            obstacles=obstacles
        )

        # State machine initialization
        self.state = CleanerState.WAITING_FOR_LANE
        self.current_pose = None
        self.current_task = None
        self.current_path = []
        self.current_path_idx = 0
        self.cleaning_start_time = 0.0
        self.waiting_for_service = False
        self.released_cleaner_lane = 0
        self.verify_deadline = 0.0
        self.verify_timeout = 15.0
        # Defects already preempted for: prevents the queue callback from re-interrupting
        # the same CRITICAL defect on every tick and growing the context stack without bound.
        self.preempted_defects = set()

        # Context stack for emergency preemption
        self.context_stack = []

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/pose', self._pose_cb, 10)
        self.queue_sub = self.create_subscription(
            TaskQueue, '/task_queue', self._queue_cb, 10)
        self.lane_coord_sub = self.create_subscription(
            LaneCoordination, '/lane_coordination/events', self._lane_coord_cb, 10)

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, f'/{self.robot_id}/target_goal', 10)
        self.tool_pub = self.create_publisher(
            Bool, f'/{self.robot_id}/tool_cmd', 10)
        self.belief_pub = self.create_publisher(
            BeliefUpdate, '/belief_updates', 20)

        # Service Client for Verification Handshake
        self.complete_cli = self.create_client(TaskComplete, '/task_complete')

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self._state_machine_loop)
        self.get_logger().info(
            f'[CLEANER] ToolController V2 initialized for {self.robot_id}. '
            f'State: {self.state.value} (waiting for Lane 1 release).'
        )

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
                with open(path, 'r') as f:
                    return yaml.safe_load(f)
        return {}

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)

    def _lane_coord_cb(self, msg: LaneCoordination):
        self.released_cleaner_lane = max(self.released_cleaner_lane, msg.released_cleaner_lane)
        if self.state == CleanerState.WAITING_FOR_LANE and self.released_cleaner_lane >= 1:
            self.state = CleanerState.IDLE
            self.get_logger().info(
                f"[COORDINATION] Cleaner transitioned: WAITING_FOR_LANE -> IDLE (Lane {self.released_cleaner_lane} released)."
            )

    def _queue_cb(self, msg: TaskQueue):
        if not self.current_pose or not msg.tasks:
            return

        # Check for immediate CRITICAL emergency defect in the queue.
        # Never preempt mid-handshake: the in-flight /task_complete callback would
        # otherwise clobber the emergency task it finds in self.current_task.
        if self.state != CleanerState.VERIFYING:
            for task in msg.tasks:
                if task.emergency_flag or (task.risk_level == 'CRITICAL'):
                    if task.status in ('VERIFIED_CLEAN', 'CLEANING'):
                        continue
                    if task.defect_id in self.preempted_defects:
                        continue
                    if self.current_task is not None and self.current_task.defect_id == task.defect_id:
                        continue
                    self._trigger_emergency_interruption(task)
                    return

        # Normal task dispatch when IDLE
        if self.state in (CleanerState.IDLE, CleanerState.WAITING_FOR_LANE):
            for task in msg.tasks:
                if task.status in ('QUEUED', 'RE_QUEUED', 'DETECTED') and task.eligible:
                    self._dispatch_task(task)
                    break

    def _trigger_emergency_interruption(self, emergency_task: TaskItem):
        """Preempts ongoing task, pushes context to stack, and navigates to emergency defect."""
        if self.current_task and self.current_task.defect_id == emergency_task.defect_id:
            return  # Already addressing this emergency

        self.get_logger().warning(
            f"[EMERGENCY] CRITICAL DEFECT DETECTED: {emergency_task.defect_id} (sev={emergency_task.severity:.2f}) "
            f"in Lane {emergency_task.lane_id}! Interrupting current cleaner activity."
        )

        # Halt cleaning brush
        self._set_tool(False)

        # Push current context to stack if active
        if self.state in (CleanerState.NAVIGATING, CleanerState.CLEANING) and self.current_task:
            context = {
                'task': self.current_task,
                'state': self.state,
                'path': self.current_path,
                'path_idx': self.current_path_idx,
            }
            self.context_stack.append(context)
            self.get_logger().info(
                f"[EMERGENCY] Saved context for task {self.current_task.defect_id} to stack "
                f"(depth: {len(self.context_stack)})."
            )

        # Plan fresh A* path to emergency defect from current position
        gx, gy = emergency_task.position.x, emergency_task.position.y
        path = self.grid_planner.plan_path(self.current_pose, (gx, gy))
        path_len = self.grid_planner.path_length(path)
        self._warn_if_plan_failed(emergency_task.defect_id)

        self.preempted_defects.add(emergency_task.defect_id)
        self.current_task = emergency_task
        self.current_path = path
        self.current_path_idx = 0
        self.state = CleanerState.EMERGENCY_NAVIGATING

        self.get_logger().warning(
            f"[EMERGENCY] State -> EMERGENCY_NAVIGATING. Executing A* path to {emergency_task.defect_id} "
            f"({len(path)} waypoints, length={path_len:.2f}m)."
        )

        # Dispatch first waypoint
        self._dispatch_current_waypoint()

    def _dispatch_task(self, task: TaskItem):
        """Initiates normal cleaning mission for eligible task."""
        gx, gy = task.position.x, task.position.y
        path = self.grid_planner.plan_path(self.current_pose, (gx, gy))
        path_len = self.grid_planner.path_length(path)
        self._warn_if_plan_failed(task.defect_id)

        self.current_task = task
        self.current_path = path
        self.current_path_idx = 0
        self.state = CleanerState.NAVIGATING

        self.get_logger().info(
            f"[CLEANER] Dispatched task {task.defect_id} ({task.defect_type}) Lane {task.lane_id} | "
            f"A* Path: {len(path)} waypoints, length={path_len:.2f}m."
        )

        # Dispatch first waypoint
        self._dispatch_current_waypoint()

    def _warn_if_plan_failed(self, defect_id):
        if self.grid_planner.last_plan_failed:
            self.get_logger().warning(
                f"[CLEANER] A* found no free route to {defect_id}; falling back to a direct line "
                "that is not obstacle-checked."
            )

    def _dispatch_current_waypoint(self):
        if self.current_path_idx < len(self.current_path):
            wx, wy = self.current_path[self.current_path_idx]
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'hull_world'
            goal_msg.pose.position.x = float(wx)
            goal_msg.pose.position.y = float(wy)
            goal_msg.pose.position.z = 0.05
            goal_msg.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_msg)

    def _set_tool(self, active: bool):
        t_msg = Bool()
        t_msg.data = active
        self.tool_pub.publish(t_msg)

    def _state_machine_loop(self):
        if not self.current_pose:
            return

        cx, cy = self.current_pose

        # 1. Navigation States: NAVIGATING or EMERGENCY_NAVIGATING
        if self.state in (CleanerState.NAVIGATING, CleanerState.EMERGENCY_NAVIGATING):
            if not self.current_task or not self.current_path:
                self.state = CleanerState.IDLE
                return

            if self.current_path_idx < len(self.current_path):
                wx, wy = self.current_path[self.current_path_idx]
                dist_to_wp = math.hypot(wx - cx, wy - cy)

                if dist_to_wp < self.arrival_dist:
                    self.current_path_idx += 1
                    if self.current_path_idx < len(self.current_path):
                        # Advance to next A* waypoint
                        self._dispatch_current_waypoint()
                    else:
                        # Final waypoint reached! Transition to active cleaning
                        is_emergency = (self.state == CleanerState.EMERGENCY_NAVIGATING)
                        self.state = CleanerState.EMERGENCY_CLEANING if is_emergency else CleanerState.CLEANING

                        if is_emergency:
                            self.get_logger().warning(
                                f"[EMERGENCY] Arrived at emergency defect {self.current_task.defect_id}. "
                                "Activating cavitation tool for priority remediation."
                            )
                        else:
                            self.get_logger().info(
                                f"[CLEANER] Arrived at defect {self.current_task.defect_id}. "
                                "Activating cavitation cleaning tool."
                            )

                        self._set_tool(True)
                        self.cleaning_start_time = time.time()

                        # Emit CLEANING status update
                        b_up = BeliefUpdate()
                        b_up.defect_id = self.current_task.defect_id
                        b_up.status = 'CLEANING'
                        b_up.severity = float(self.current_task.severity)
                        b_up.confidence = 0.95
                        b_up.lane_id = int(self.current_task.lane_id)
                        b_up.risk_level = self.current_task.risk_level
                        b_up.emergency_flag = self.current_task.emergency_flag
                        b_up.eligible_lane = True
                        b_up.assigned_robot = self.robot_id
                        b_up.stamp = self.get_clock().now().to_msg()
                        self.belief_pub.publish(b_up)

        # 2. Active Cleaning States: CLEANING or EMERGENCY_CLEANING
        elif self.state in (CleanerState.CLEANING, CleanerState.EMERGENCY_CLEANING):
            elapsed = time.time() - self.cleaning_start_time
            if elapsed >= self.clean_duration:
                # Dwell time complete
                self._set_tool(False)
                self.state = CleanerState.VERIFYING
                self.verify_deadline = time.time() + self.verify_timeout
                self.get_logger().info(
                    f"[CLEANER] Dwell time elapsed ({self.clean_duration:.1f}s) on {self.current_task.defect_id}. "
                    "Tool DEACTIVATED. Calling /task_complete service for closed-loop rescan."
                )
                self._call_task_complete()

        # 3. Verification handshake: retry until the service answers, then give up
        #    rather than stalling the whole mission in VERIFYING forever.
        elif self.state == CleanerState.VERIFYING:
            # Deadline is checked first so a request whose future never resolves
            # is bounded too, not just a service that is slow to come up.
            if time.time() >= self.verify_deadline:
                stalled_id = self.current_task.defect_id if self.current_task else 'unknown'
                self.get_logger().error(
                    f"[CLEANER] /task_complete did not complete within {self.verify_timeout:.0f}s on {stalled_id}. "
                    "Abandoning verification and returning to IDLE so the mission can continue."
                )
                self._release_task()
            elif not self.waiting_for_service:
                self._call_task_complete()

    def _call_task_complete(self):
        if self.current_task is None:
            self.get_logger().error('[CLEANER] Verification requested with no active task; releasing cleaner.')
            self._release_task()
            return

        if not self.complete_cli.service_is_ready():
            self.get_logger().warning(
                '[CLEANER] /task_complete service not available; retrying...',
                throttle_duration_sec=2.0)
            return

        self.waiting_for_service = True
        req = TaskComplete.Request()
        req.defect_id = self.current_task.defect_id
        req.robot_id = self.robot_id
        req.reported_success = True

        future = self.complete_cli.call_async(req)
        future.add_done_callback(lambda f: self._complete_done_cb(f, self.current_task.defect_id))

    def _complete_done_cb(self, future, d_id):
        self.waiting_for_service = False
        try:
            res = future.result()
            if res.verification_triggered:
                self.get_logger().info(
                    f"[VERIFICATION] Handshake confirmed for {d_id}: {res.message}"
                )
            else:
                self.get_logger().warning(
                    f"[VERIFICATION] Handshake reported verification not triggered for {d_id}."
                )
        except Exception as e:
            self.get_logger().error(f'[CLEANER] /task_complete service call failed: {e}')

        # Check if we were handling an emergency and have a suspended context to resume
        if self.context_stack:
            prev_context = self.context_stack.pop()
            self.get_logger().info(
                f"[EMERGENCY] Resuming suspended context for task {prev_context['task'].defect_id} "
                f"from current cleaner position."
            )
            # Replan fresh A* path from current position to suspended task
            self._dispatch_task(prev_context['task'])
        else:
            self._release_task()

    def _release_task(self):
        """Clears the active task and returns the cleaner to the dispatch pool."""
        self.waiting_for_service = False
        self.current_task = None
        self.current_path = []
        self.current_path_idx = 0
        self.state = CleanerState.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ToolController()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # Never swallow a fatal error: a silent exit makes a dead node look healthy.
        if node is not None:
            node.get_logger().fatal('[CLEANER] Fatal error, shutting down:\n' + traceback.format_exc())
        else:
            traceback.print_exc()
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
