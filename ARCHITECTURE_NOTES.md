# HullSentinal Architecture & Codebase Notes

Comprehensive audit of HullSentinal V1 architecture prior to the V2 Cognitive Multi-Robot Upgrade.

---

## 1. System Topology & Data Flow Overview

```
                        [run_demo.sh]
                              │
                              ▼
                 [hull_full.launch.py]
                              │
     ┌────────────────────────┴────────────────────────┐
     │                                                 │
     ▼                                                 ▼
[view_robots.launch.py]                        [sim_engine_node]
(Publishes URDFs to robot_description)          (Ground truth physics & defects)
                                                       │
                                   ┌───────────────────┴───────────────────┐
                                   │ /defect_detections                    │ /mapper_1/pose, /cleaner_1/pose
                                   ▼                                       ▼
                          [belief_map_node]                       [crawler_controller]
                        (SQLite DB + Cache)                      (Lawnmower / P2P Motion)
                                   │                                       ▲
                   /belief_map/    │ /belief_updates                       │ /cmd_vel_raw
                 active_defects    ▼                                       │
                               [utility_scorer]                    [safety_barrier]
                             (Heuristic Priority)                  (Inter-robot & perimeter)
                                   │                                       │
                                   │ /task_queue                           ▼ /cmd_vel
                                   ▼                               [sim_engine_node]
                           [tool_controller]
                         (Nav -> Clean Machine)
                                   │
                                   │ /task_complete srv
                                   ▼
                        [verification_monitor]
                                   │
                                   │ /rescan_request srv
                                   ▼
                           [sim_engine_node]
                                   │
                                   │ /belief_updates
                                   ▼
                         [belief_map_node]
                                   │
                                   ▼
                         [marker_publisher] ──> /defects, /belief_map/heatmap, /task_queue_marker
                                   │
                                   ▼
                                [RViz2] / [demo_snapshot]
```

---

## 2. Exhaustive Node Directory

### 2.1 `sim_engine_node` (`hull_sim/sim_engine_node.py`)
- **Role:** Simulates multi-robot kinematics, surface contact constraints, ground truth defects, sensor perception footprint, defect degradation physics, and verification rescan.
- **Publishers:**
  - `/{rid}/pose` (`geometry_msgs/msg/PoseStamped`): Robot pose on hull manifold ($Z=0.05$).
  - `/{rid}/odom` (`nav_msgs/msg/Odometry`): Robot odometry.
  - `/{rid}/status` (`hull_msgs/msg/RobotStatus`): Battery, role, tool state, task ID.
  - `/defect_detections` (`hull_msgs/msg/DefectDetection`): Emitted when mapper sensor cone covers a defect.
  - `/hull_sim/mission_status` (`hull_msgs/msg/MissionStatus`): Total, cleaned, pending counts.
  - `/hull_sim/defect_states` (`std_msgs/msg/String`): JSON string of ground-truth defect state histories.
  - `tf` (`tf2_msgs/msg/TFMessage`): Broadcasts `hull_world -> {rid}/base_footprint`.
- **Subscribers:**
  - `/{rid}/cmd_vel` (`geometry_msgs/msg/Twist`): Ingests filtered velocity commands.
  - `/{rid}/tool_cmd` (`std_msgs/msg/Bool`): Ingests cleaner cavitation tool on/off command.
  - `/belief_updates` (`hull_msgs/msg/BeliefUpdate`): Syncs ground truth defect state with cognition updates.
- **Service Servers:**
  - `/rescan_request` (`hull_msgs/srv/RescanRequest`): Evaluates ground truth residual defect at target $(x, y)$.
- **Timers:**
  - `sim_timer`: Period `dt` (default 0.05s / 20 Hz) updates kinematics, checks collisions, degrades active cleaning defects, and performs perception checks.
- **Parameters:**
  - `num_cleaners` (int, default 1)
  - `dt` (double, default 0.05)
  - `current_disturbance_x`, `current_disturbance_y` (double, default 0.0)
  - `sensor_noise_std` (double, default 0.03)
  - `cleaning_failure_rate` (double, default 0.0)
  - `cleaning_rate` (double, default 1.50)
  - `tool_radius` (double, default 0.45)
  - `detection_radius` (double, default 2.20)
  - `scenario_name` (string, default 'nominal')

### 2.2 `belief_map_node` (`hull_swarm/belief_map_node.py`)
- **Role:** Centralized probabilistic world state engine using SQLAlchemy ORM (SQLite in-memory or file). Handles defect lifecycle: `UNDETECTED -> DETECTED -> QUEUED -> CLEANING -> VERIFIED_CLEAN` (or `RE_QUEUED`).
- **Publishers:**
  - `/belief_updates` (`hull_msgs/msg/BeliefUpdate`): Broadcasts status, severity, confidence changes.
  - `/belief_map/active_defects` (`hull_msgs/msg/TaskQueue`): Dispatches all uncleaned defects to the utility scorer.
- **Subscribers:**
  - `/defect_detections` (`hull_msgs/msg/DefectDetection`): Inserts new defect or applies Bayesian update.
  - `/belief_updates` (`hull_msgs/msg/BeliefUpdate`): Synchronizes state changes from external nodes.
  - `/task_queue` (`hull_msgs/msg/TaskQueue`): Transitions `DETECTED -> QUEUED` upon task ranking.
- **Timers:**
  - `timer`: 0.5s (2 Hz) syncs and publishes `/belief_map/active_defects`.
- **Parameters:**
  - `db_path` (string, default ':memory:')

### 2.3 `utility_scorer` (`hull_swarm/utility_scorer.py`)
- **Role:** Computes multi-attribute dynamic utility for all active defects:
  $$U(d) = (S_d \cdot W_{\text{type}} \cdot C_d) - \alpha \cdot \text{dist} + \text{Bonus}_{\text{requeue}}$$
  Sorts tasks in descending order using a Python `heapq` and dispatches prioritized queue.
- **Publishers:**
  - `/task_queue` (`hull_msgs/msg/TaskQueue`): Ranked array of `TaskItem`s with computed utility.
- **Subscribers:**
  - `/belief_map/active_defects` (`hull_msgs/msg/TaskQueue`): Ingests active defects.
  - `/{cleaner_id}/pose` (`geometry_msgs/msg/PoseStamped`): Tracks cleaner position for distance calculation.
- **Timers:**
  - `timer`: 0.5s (2 Hz) re-scores and publishes `/task_queue`.
- **Parameters:**
  - `weight_crack` (double, default 1.0)
  - `weight_rust` (double, default 0.70)
  - `weight_biofouling` (double, default 0.40)
  - `distance_alpha` (double, default 0.05)
  - `requeue_escalation` (double, default 0.15)
  - `cleaner_robot_id` (string, default 'cleaner_1')

### 2.4 `verification_monitor` (`hull_swarm/verification_monitor.py`)
- **Role:** Implements closed-loop verification. When cleaner signals `/task_complete`, requests rescan from mapper/simulation. Confirms residual severity $\le$ threshold to mark `VERIFIED_CLEAN` or escalates to `RE_QUEUED`.
- **Publishers:**
  - `/belief_updates` (`hull_msgs/msg/BeliefUpdate`): Emits `VERIFIED_CLEAN` or `RE_QUEUED`.
- **Service Servers:**
  - `/task_complete` (`hull_msgs/srv/TaskComplete`): Called by cleaner tool controller.
- **Service Clients:**
  - `/rescan_request` (`hull_msgs/srv/RescanRequest`): Dispatched to simulation / mapper.
- **Parameters:**
  - `residual_threshold` (double, default 0.08)

### 2.5 `crawler_controller` (`hull_control/crawler_controller.py`)
- **Role:** Generates surface motion commands.
  - For `mapper`: Lawnmower sweep through predefined waypoints.
  - For `cleaner`: Proportional heading & pure pursuit guidance to `/{robot_id}/target_goal`.
  - Implements proactive perimeter braking near geofence boundaries.
- **Publishers:**
  - `/{robot_id}/cmd_vel_raw` (`geometry_msgs/msg/Twist`): Unfiltered velocity commands.
- **Subscribers:**
  - `/{robot_id}/pose` (`geometry_msgs/msg/PoseStamped`): Current robot pose.
  - `/{robot_id}/target_goal` (`geometry_msgs/msg/PoseStamped`): Dispatched target position.
- **Timers:**
  - `timer`: 0.05s (20 Hz) executes the control loop.
- **Parameters:**
  - `robot_id` (string, default 'mapper_1')
  - `role` (string, default 'mapper')
  - `max_linear_speed` (double, default 0.6)
  - `max_angular_speed` (double, default 1.2)
  - `goal_tolerance` (double, default 0.25)
  - `x_min`, `x_max`, `y_min`, `y_max` (doubles, default 0.8, 19.2, 0.8, 5.2)

### 2.6 `tool_controller` (`hull_control/tool_controller.py`)
- **Role:** High-level cleaning state machine:
  `IDLE -> NAVIGATING -> CLEANING -> REPORTING -> IDLE`.
  Dispatches `target_goal` to crawler controller, publishes `tool_cmd=True` upon arrival, broadcasts `CLEANING` belief update, and triggers `/task_complete` service.
- **Publishers:**
  - `/{robot_id}/target_goal` (`geometry_msgs/msg/PoseStamped`): Navigation waypoint.
  - `/{robot_id}/tool_cmd` (`std_msgs/msg/Bool`): Activates/deactivates cavitation brush.
  - `/belief_updates` (`hull_msgs/msg/BeliefUpdate`): Broadcasts `status=CLEANING`.
- **Subscribers:**
  - `/{robot_id}/pose` (`geometry_msgs/msg/PoseStamped`): Tracks cleaner pose.
  - `/task_queue` (`hull_msgs/msg/TaskQueue`): Picks top-priority `QUEUED` task.
- **Service Clients:**
  - `/task_complete` (`hull_msgs/srv/TaskComplete`): Reports cleaning completion.
- **Timers:**
  - `timer`: 0.1s (10 Hz) updates state machine.
- **Parameters:**
  - `robot_id` (string, default 'cleaner_1')
  - `clean_duration` (double, default 0.90)
  - `arrival_distance` (double, default 0.40)

### 2.7 `safety_barrier` (`hull_control/safety_barrier.py`)
- **Role:** Control barrier certificates & collision prevention. Scales down or halts velocity commands if inter-robot distance $< \text{safety\_radius}$ ($0.55$m), halts and turns if $< 0.35$m, and repels robots away from hull edges.
- **Publishers:**
  - `/{rid}/cmd_vel` (`geometry_msgs/msg/Twist`): Safe commands sent to simulation.
- **Subscribers:**
  - `/{rid}/pose` (`geometry_msgs/msg/PoseStamped`): All robot positions.
  - `/{rid}/cmd_vel_raw` (`geometry_msgs/msg/Twist`): Raw controller outputs.
- **Timers:**
  - `timer`: 0.05s (20 Hz) filtering loop.
- **Parameters:**
  - `robots` (string array, default `['mapper_1', 'cleaner_1']`)
  - `safety_radius` (double, default 0.55)
  - `x_min`, `x_max`, `y_min`, `y_max` (doubles, default 0.8, 19.2, 0.8, 5.2)

### 2.8 `geofence_node` (`hull_control/geofence_node.py`)
- **Role:** Perimeter watchdog node monitoring robot poses against hard hull boundaries ($[0.3, 19.7] \times [0.3, 5.7]$). Increments `out_of_bounds_count` upon breach.
- **Publishers:**
  - `/geofence_status` (`std_msgs/msg/String`): JSON string with `out_of_bounds_count`, `all_in_bounds`, and robot status.
- **Subscribers:**
  - `/{rid}/pose` (`geometry_msgs/msg/PoseStamped`)
- **Timers:**
  - `timer`: 0.1s (10 Hz) monitor loop.
- **Parameters:**
  - `robots`, `x_min`, `x_max`, `y_min`, `y_max`, `hard_x_min`, `hard_x_max`, `hard_y_min`, `hard_y_max`

### 2.9 `marker_publisher` (`hull_bringup/marker_publisher.py`)
- **Role:** Visual presentation layer.
- **Publishers:**
  - `/defects` (`visualization_msgs/msg/MarkerArray`): Defect cubes (crack), cylinders (rust), spheres (biofouling), state colors, progress bars, and text labels.
  - `/belief_map/heatmap` (`visualization_msgs/msg/MarkerArray`): Per-cell belief & utility grid.
  - `/task_queue_marker` (`visualization_msgs/msg/Marker`): Text HUD of ranked queue.
  - `/mapper_1/sensor_fov` (`visualization_msgs/msg/Marker`): Cyan translucent cone.
  - `/{rid}/path_marker` (`visualization_msgs/msg/Marker`): Trajectory line strip.
  - `/{rid}/goal_marker` (`visualization_msgs/msg/Marker`): 3D Arrow.
  - `/visualization_marker_array` (`visualization_msgs/msg/MarkerArray`): Composite array.
- **Subscribers:**
  - `/{rid}/pose`, `/{rid}/target_goal`, `/{rid}/tool_cmd`, `/defect_detections`, `/belief_updates`, `/task_queue`.
- **Timers:**
  - `timer`: 0.1s (10 Hz) broadcast timer.

### 2.10 `demo_snapshot` (`hull_bringup/demo_snapshot.py`)
- **Role:** Automated headless visual capture generating high-resolution Matplotlib snapshots:
  - `docs/screenshot_start.png` (inspection start)
  - `docs/screenshot_mid_cleaning.png` (active cleaning)
  - `docs/screenshot_end.png` (verified clean)
  - `docs/simulation_screenshot.png` (continuous latest)
- **Subscribers:**
  - `/mapper_1/pose`, `/cleaner_1/pose`, `/defects`, `/cleaner_1/tool_cmd`.
- **Timers:**
  - `timer`: 0.5s check & render loop.

---

## 3. Current World, Defect, and Spawn Definitions

- **World Frame:** `hull_world`.
- **Hull Geometry:** 20.0m length $\times$ 6.0m height $\times$ 0.12m thickness.
- **Water Plane:** 28.0m length $\times$ 12.0m height behind the hull ($Z=-0.25$).
- **Safety Borders:** Rub rails at $Y=0$, $Y=6.0$, $X=0$, $X=20.0$.
- **Robot Spawns:**
  - `mapper_1`: $(1.5, 1.5, 0.0)$
  - `cleaner_1`: $(1.5, 4.5, 0.0)$
- **Defects (9 Ground Truth in V1):**
  - Cracks: `defect_crack_01` (4.0, 2.0), `defect_crack_02` (8.5, 4.5), `defect_crack_03` (14.0, 1.5).
  - Rust: `defect_rust_01` (3.0, 4.0), `defect_rust_02` (11.0, 3.0), `defect_rust_03` (16.5, 4.8).
  - Biofouling: `defect_bio_01` (6.0, 1.2), `defect_bio_02` (12.5, 5.0), `defect_bio_03` (17.5, 2.2).

---

## 4. Current Mapper Lanes & Cleaner Task Choice Mechanisms

### Mapper Motion Today:
- Uses a fixed hard-coded list of 17 waypoints in `crawler_controller.py`:
  - $(2.0, 1.5) \to (18.0, 1.5)$
  - $(18.0, 3.0) \to (2.0, 3.0)$
  - $(2.0, 4.5) \to (18.0, 4.5)$
- When $\text{dist} < 0.25$, waypoint index increments.
- There is NO explicit lane state machine (`MAPPING_LANE_1`, etc.), NO lane coordination with the cleaner, NO obstacle detours, and NO coverage tracking.

### Cleaner Task Choice Today:
- Subscribes to `/task_queue`.
- When in `IDLE`, iterates over tasks and greedily picks the first task with status `QUEUED` or `RE_QUEUED`.
- Distance in utility formula is straight-line Euclidean distance:
  $$\text{dist} = \sqrt{(x_{\text{task}} - x_{\text{cleaner}})^2 + (y_{\text{task}} - y_{\text{cleaner}})^2}$$
- Cleaner does NOT check which lane the defect is in.
- Cleaner starts immediately upon task detection, regardless of mapper progress.
- There are no static obstacles or 2D occupancy grid A* path calculations.

---

## 5. Marker Production & Label Architecture Limitations Today

1. **Defects MarkerArray:**
   - In `marker_publisher.py`, an integer counter `m_id` is incremented for each geometry marker, text marker, and progress bar marker.
   - IDs are dynamic based on loop order, rather than a fixed hash/index map per defect.
   - In earlier iterations, `sim_engine_node` or other nodes could potentially emit duplicate texts.
2. **RViz Layout:**
   - Pre-configured in `rviz_config.rviz` with fixed frame `hull_world`, camera focal point at $(10, 3, 0)$, and distance 19.5m.

---

## 6. Execution Flow Today

1. `./run_demo.sh` sources ROS 2 environment and executes `ros2 launch hull_bringup hull_full.launch.py`.
2. `view_robots.launch.py` starts 3 `robot_state_publisher` nodes for `/hull`, `/mapper_1`, and `/cleaner_1`.
3. `sim_engine_node` spawns robots and initializes 9 ground truth defects.
4. `crawler_controller` drives `mapper_1` along the hard-coded sweep waypoints.
5. As mapper's detection radius ($2.2$m) covers defects, `sim_engine_node` publishes `/defect_detections`.
6. `belief_map_node` receives detections, stores them as `DETECTED`, and emits `BeliefUpdate`.
7. `utility_scorer` computes utility scores and publishes `/task_queue` (marking items `QUEUED`).
8. `tool_controller` picks the top task, commands cleaner crawler, drives to the site, activates tool (`CLEANING`), and calls `/task_complete`.
9. `verification_monitor` triggers `/rescan_request` to `sim_engine_node`. If residual $< 0.08$, emits `VERIFIED_CLEAN`.
10. `marker_publisher` updates visual state in RViz; `demo_snapshot` writes PNG snapshots to `docs/`.
