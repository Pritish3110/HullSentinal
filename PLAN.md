# HullSentinal — System Architecture & Implementation Plan

**Project:** Belief-Based Task Planning for Multi-Robot Ship Hull Cleaning  
**Environment:** Ubuntu 26.04 (Resolute) | ROS 2 Lyrical | Gazebo Sim & Headless Manifold Engine | Python 3.14  
**Workspace:** `./ws` (under `/home/pritish_3110/Personal Files/Syllabus/Projects/HullSentinal/ws`)  
**Reference Model:** `/home/pritish_3110/Personal Files/Syllabus/Projects/Capstone_project/usv_swarm_ws/` (Read-only)

---

## 1. Executive Summary & Problem Formulation

Ship hull degradation (biofouling, corrosion/rust, structural cracks) increases hydrodynamic drag and fuel consumption and jeopardizes vessel structural integrity. This project implements a closed-loop cognitive robotics pipeline for autonomous multi-robot ship hull inspection and remediation.

### The Core Cognitive Gap
Conventional multi-robot inspection either follows static pre-planned routes or distance-greedy bidding, marking tasks completed upon visiting coordinates without validating cleanliness. HullSentinal addresses this through a three-pillared cognitive architecture:
1. **Belief-State Representation:** Defects (crack, rust, biofouling) are modeled not merely as coordinates, but as probabilistic belief states encompassing type, continuous severity estimate $S \in [0, 1]$, detection confidence $C \in [0, 1]$, and spatial coordinates $(x, y, z)$ on the hull manifold.
2. **Severity-Aware Utility Scorer:** Tasks are prioritized dynamically via a transparent, multi-attribute utility function:
   $$U(d, t) = \big(S(d) \cdot W_{\text{risk}}(\text{type}(d)) \cdot C(d)\big) - \alpha \cdot D(\mathbf{p}_{\text{cleaner}}, \mathbf{p}_d)$$
   where structural risks are ranked ($\text{crack}: 1.0 > \text{rust}: 0.7 > \text{biofouling}: 0.4$), balanced against transit cost $\alpha \cdot D$.
3. **Closed-Loop Verification & Re-Queueing:** Cleaning robot completion triggers a re-scan request to the mapper/inspector robot. The re-scan computes area reduction and residual defect probability. If cleaning is incomplete or defect severity worsened, the belief map updates and the task is immediately re-queued.

---

## 2. ROS 2 Workspace Architecture (`./ws/src`)

The workspace adheres to enterprise ROS 2 standard packaging, decoupled into 7 distinct packages:

```
ws/
└── src/
    ├── hull_msgs/            # Custom interfaces (Messages, Services, Actions)
    ├── hull_description/     # URDF/Xacro models, hull mesh, collision geometries
    ├── hull_sim/             # Gazebo worlds, manifold physics engine, defect spawner
    ├── hull_control/         # Adhesion crawler kinematics, trajectory tracking, tool state machines
    ├── hull_swarm/           # Cognition layer: Belief Map, Utility Scorer, Task Allocator, Verifier
    ├── hull_benchmarks/      # Automated multi-seed benchmark suites, CSV loggers, plotting
    └── hull_bringup/         # System launch files, YAML scenario configs, RViz2 configs
```

### Package 2.1: `hull_msgs` (ament_cmake)
Declares all DDS interfaces:
- **`msg/DefectDetection.msg`**: `string defect_id`, `string defect_type`, `float32 severity`, `float32 confidence`, `geometry_msgs/Point position`, `builtin_interfaces/Time stamp`
- **`msg/TaskItem.msg`**: `string defect_id`, `string defect_type`, `float32 utility_score`, `geometry_msgs/Point position`, `uint8 status`
- **`msg/TaskQueue.msg`**: `std_msgs/Header header`, `TaskItem[] tasks`
- **`msg/BeliefUpdate.msg`**: `string defect_id`, `string status`, `float32 severity`, `float32 confidence`, `uint32 rescan_count`
- **`msg/RobotStatus.msg`**: `string robot_id`, `string role`, `geometry_msgs/Pose pose`, `string current_task_id`, `string tool_state`, `float32 battery_pct`
- **`msg/MissionStatus.msg`**: `std_msgs/Header header`, `uint32 total_defects`, `uint32 pending_count`, `uint32 in_progress_count`, `uint32 cleaned_count`, `uint32 requeued_count`, `float32 elapsed_time_sec`
- **`srv/TaskComplete.srv`**: `string defect_id`, `string robot_id`, `bool reported_success` $\rightarrow$ `bool verification_triggered`, `string message`
- **`srv/RescanRequest.srv`**: `string defect_id`, `geometry_msgs/Point position`, `float32 scan_radius` $\rightarrow$ `bool defect_present`, `float32 residual_severity`, `float32 confidence`

### Package 2.2: `hull_description` (ament_cmake)
- Models the **Mapping Robot** (`mapper_robot.urdf.xacro`): Compact magnetic track rover with an RGB-D camera sensor mount, LED lighting, and TF tree.
- Models the **Cleaning Robot** (`cleaner_robot.urdf.xacro`): Heavy-duty dual-track crawler with rotary scrubbing/waterjet tool actuator.
- Ship hull static surface model (`ship_hull.urdf.xacro` and visual/collision geometries).

### Package 2.3: `hull_sim` (ament_python)
- Dual-mode simulation:
  1. **Gazebo Sim Integration:** SDF world file with ship hull surface, lighting, and camera sensor plugins.
  2. **High-Fidelity Headless Manifold Simulator (`sim_engine_node.py`):** Fully deterministic simulation of multi-robot kinematics on 2D/3D hull manifold, defect degradation dynamics, realistic sensor noise models, and hydrodynamic current disturbances. Capable of headless evaluation at 20-50x real-time speed for statistical benchmarking.

### Package 2.4: `hull_control` (ament_python)
- **`crawler_controller.py`**: Manifold navigation controller steering crawler towards defect coordinates using magnetic adhesion kinematics.
- **`tool_controller.py`**: Cleaning tool state machine (`IDLE` $\rightarrow$ `NAVIGATING` $\rightarrow$ `ALIGNING` $\rightarrow$ `CLEANING` $\rightarrow$ `REPORTING` $\rightarrow$ `IDLE`).
- **`safety_barrier.py`**: Inter-robot collision avoidance ensuring mapper and cleaners maintain safe separation distances.

### Package 2.5: `hull_swarm` (ament_python)
- **`belief_map_node.py`**: Persistent belief-state database (SQLAlchemy SQLite ORM) tracking defect lifecycles: `PENDING` $\rightarrow$ `QUEUED` $\rightarrow$ `IN_PROGRESS` $\rightarrow$ `VERIFYING` $\rightarrow$ `DONE` or `RE_QUEUED`.
- **`utility_scorer.py`**: Computes real-time dynamic utilities across active defects based on structural risk weights, detection confidence, and Euclidean proximity.
- **`task_queue_manager.py`**: Max-heap task priority dispatcher supporting heterogeneous multi-robot allocation.
- **`verification_monitor.py`**: Manages verification handshake upon `/task_complete`, requests `/rescan_request`, and governs probabilistic re-queueing decisions.

### Package 2.6: `hull_benchmarks` (ament_python)
- Automated test harness running 6 scenarios across 5 random seeds (30 evaluation runs total).
- Metric aggregation: Task completion vs utility ranking correlation, time-to-clean-all, verification catch rate, re-queue counts, collisions, CPU load.
- Plotting engine generating comparison curves between Prioritized Belief-Based Planning and Naive FIFO Baseline.
- Automatic Markdown generator for `BENCHMARK_REPORT.md`.

### Package 2.7: `hull_bringup` (ament_python)
- Launch scripts:
  - `hull_full.launch.py`: Complete stack bringup.
  - `scenario.launch.py`: Parametrized scenario runner.
  - `headless_eval.launch.py`: Fast-time headless batch evaluator.
- RViz2 configuration: Displaying hull point clouds, robot models, TF trees, and state-coded defect markers (Red=Pending, Yellow=In Progress, Blue=Verifying, Green=Done, Orange=Re-queued).

---

## 3. Detailed Data Flow & State Machine

```
[Defect Spawner / World]
       │ (Ground truth defects)
       ▼
[Mapper Robot (Perception Node)]
       │ Publishes: /defect_detections (DefectDetection)
       ▼
[Belief Map Node] ◀──────────────────────────────────────────┐
       │ (Upserts belief states in SQLite)                    │
       ▼                                                      │
[Utility Scorer]                                              │
       │ Calculates: U = S * W_risk * C - α * Dist            │
       ▼                                                      │
[Task Queue Manager]                                          │
       │ Publishes: /task_queue (TaskQueue)                   │
       ▼                                                      │
[Cleaner Robot(s) Node]                                       │
       │ Navigates to highest priority task                   │
       │ Executes cleaning action                             │
       │ Calls service: /task_complete                        │
       ▼                                                      │
[Verification Monitor Node]                                   │
       │ Triggers service: /rescan_request                    │
       ▼                                                      │
[Mapper Robot Node] (Re-scans site)                           │
       │ Returns: residual severity & confidence              │
       ▼                                                      │
[Verification Monitor Node] ──────────────────────────────────┘
       If clean: Status -> DONE
       If residual > threshold: Status -> RE_QUEUED (Score elevated)
```

---

## 4. Test Scenarios Definition (Phase 5 Gate)

Six benchmark scenarios will be configured in `hull_bringup/config/`:
1. **Scenario 1: Nominal Inspection & Cleaning** (1 Mapper, 1 Cleaner, 9 defects across crack/rust/biofouling, standard adhesion).
2. **Scenario 2: Swarm Scalability** (1 Mapper, 3 Cleaners, 18 defects, parallel auction allocation).
3. **Scenario 3: Obstacle-Cluttered Hull** (Sacrificial anodes, welds, sea chest openings requiring collision avoidance detour).
4. **Scenario 4: High Sensor Noise** (Gaussian noise on camera confidence $\sigma = 0.3$, false positive clutter).
5. **Scenario 5: Cleaner Actuator Failure** (Cleaner 1 suffers tool failure at $t = 30$s, dynamic re-allocation to Cleaner 2).
6. **Scenario 6: Hydrodynamic Current Disturbance** (Marine current drag force $F_{\text{current}}$ inducing drift on magnetic crawlers).

---

## 5. Verification & Benchmark Metrics (Phase 6 Gate)

The benchmark suite logs per-seed CSV records:
- **Spearman Rank Correlation ($r_s$):** Correlation between initial utility ranking and execution completion sequence.
- **Time to Complete ($T_{\text{mission}}$):** Wall-clock / simulated seconds to resolve all defects.
- **Verification Catch Rate ($R_{\text{catch}}$):** Ratio of successfully caught incomplete cleans to total verification events.
- **Total Re-queue Count ($N_{\text{requeue}}$):** Number of times imperfect cleans were rescheduled.
- **Inter-Robot Collisions ($N_{\text{collision}}$):** Validating safety barrier constraint violations.
- **CPU & Memory Footprint:** Average compute utilization during continuous ROS 2 pub/sub loops.

---

## 6. Execution Roadmap & Gate Criteria

| Phase | Target Deliverable | Gate Criteria |
|---|---|---|
| **Phase 1** | `PLAN.md`, `PROGRESS.md`, `BLOCKERS.md` | Covers every requirement in `project.md`. |
| **Phase 2** | Scaffold `./ws`, create 7 packages, build via colcon | `colcon build --symlink-install` succeeds with 0 errors. |
| **Phase 3** | Spawn robots, sensor plugins, URDFs, controllers | All bots spawn, TF frames publish, `/cmd_vel` response verified. |
| **Phase 4** | Implement belief map, scorer, queue manager, verifier | Unit tests pass, 1 full nominal mission runs end-to-end. |
| **Phase 5** | Implement 6 scenario YAML configs & launch files | All 6 scenarios run headless to completion with 0 crashes. |
| **Phase 6** | Execute 6 scenarios $\times$ 5 seeds (30 runs), log CSVs | Generate plots & `BENCHMARK_REPORT.md` with full analytics. |
| **Phase 7** | Create `./run_demo.sh` for interactive Gazebo + RViz2 | Single command runs complete interactive visual simulation. |
| **Phase 8** | Final clean build, smoke test, write `README.md` | Zero regressions. Generate `DONE.md`. |
