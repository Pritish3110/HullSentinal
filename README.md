# HullSentinal: Belief-Based Task Planning for Multi-Robot Ship Hull Cleaning

**Cognitive Robotics Project — Closed-Loop Multi-Robot Inspection & Cleaning**  
**OS:** Ubuntu 26.04 (Resolute) | **Middleware:** ROS 2 Lyrical | **Python:** 3.14  
**Workspace:** `./ws`

---

## 1. Project Overview

Ship hull degradation (biofouling, corrosion/rust, structural cracks) drastically increases hydrodynamic drag and fuel consumption and risks structural failure. Conventional inspection and cleaning systems suffer from two critical limitations:
1. **Blind Task Allocation:** Work is dispatched strictly by proximity or FIFO arrival order, ignoring defect severity and structural criticality.
2. **Open-Loop Assumption:** Tasks are marked "completed" simply upon reaching coordinates, with zero validation that the defect was actually removed.

**HullSentinal** solves this with a closed-loop cognitive robotics pipeline:
- **Probabilistic Belief Map:** Maintains continuous severity $S \in [0, 1]$, detection confidence $C \in [0, 1]$, and spatial coordinates $(x, y, z)$ on the 20m × 6m hull surface plate.
- **Dynamic Utility Scorer:** Prioritizes remediation based on structural hazard:
  $$U(d, t) = \big(S(d) \cdot W_{\text{risk}}(\text{type}(d)) \cdot C(d)\big) - \alpha \cdot D(\mathbf{p}_{\text{cleaner}}, \mathbf{p}_d) + \text{Bonus}_{\text{requeue}}$$
  where structural hazard weights follow: $\text{Crack} (1.0) > \text{Rust} (0.70) > \text{Biofouling} (0.40)$.
- **Closed-Loop Verification Monitor:** When cleaning is reported complete, a re-scan request is dispatched to the mapping robot. If residual defect exceeds threshold, the belief map updates and the task is immediately **re-queued** with escalated priority.
- **Strict Perimeter Geofence Containment:** Monitors $[0.8, 19.2] \times [0.8, 5.2]$ operational envelope on the 20m × 6m hull manifold, ensuring zero out-of-bounds drift (`out_of_bounds_count == 0`).

---

## 2. Workspace Package Structure (`./ws/src`)

| Package | Build Type | Description |
|---|---|---|
| [`hull_msgs`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_msgs) | `ament_cmake` | Custom DDS interfaces: `DefectDetection`, `TaskItem`, `TaskQueue`, `BeliefUpdate`, `RobotStatus`, `MissionStatus`, `TaskComplete.srv`, `RescanRequest.srv` |
| [`hull_description`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_description) | `ament_cmake` | URDF/Xacro models: Mapper ROV (cyan, sensor pod, dual LEDs), Cleaner crawler (orange, magnetic treads, lime rotary brush), and 20m×6m steel hull plate with ocean backplane and safety perimeter coaming |
| [`hull_sim`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_sim) | `ament_python` | High-fidelity multi-robot kinematic simulation, sensor FOV cone footprint perception, defect degradation physics, and Gazebo world |
| [`hull_control`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_control) | `ament_python` | Lawnmower sweep crawler controller, P2P proportional tracking, tool state machines, geofence boundary monitor, and control barrier certificate safety filters |
| [`hull_swarm`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_swarm) | `ament_python` | Cognition layer: SQLAlchemy SQLite Belief Map, dynamic Utility Scorer, and closed-loop Verification Monitor |
| [`hull_bringup`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_bringup) | `ament_python` | Master launch files (`hull_full.launch.py`, `scenario.launch.py`), RViz2 layout with `RobotModel` displays, marker publisher, and visual snapshot node |
| [`hull_benchmarks`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_benchmarks) | `ament_python` | Automated 6-scenario × 5-seed benchmark runner, naive FIFO baseline node, and statistical report generator |

---

## 3. High-Fidelity RViz2 Visualization & Topics

The visualization stack publishes complete 3D robot models, environmental structures, belief state heatmaps, and dynamic defect geometry:

| Topic | Type | Visual Representation |
|---|---|---|
| `/mapper_1/robot_description` | `std_msgs/String` | 3D inspection ROV body (>0.65m) in bright cyan with camera sensor pod, high-intensity LED illuminators, and magnetic adhesion tracks |
| `/cleaner_1/robot_description` | `std_msgs/String` | 3D industrial crawler body (>0.70m) in industrial orange with heavy magnetic treads, tool boom arm, and rotary lime green cavitation brush |
| `/hull/robot_description` | `std_msgs/String` | 20.0m × 6.0m marine steel hull plate, deep ocean water backplane, structural weld seams, and yellow safety rub rails |
| `/defects` | `visualization_msgs/MarkerArray` | Ground-truth defects with shapes by type (**cracks**=cubes, **rust**=discs, **biofouling**=spheres), scale proportional to severity, and text status HUD |
| `/belief_map/heatmap` | `visualization_msgs/MarkerArray` | 2D hull surface heatmap showing per-cell belief probability and utility density |
| `/task_queue_marker` | `visualization_msgs/Marker` | Real-time HUD table showing ranked tasks with utility scores and current target highlighted |
| `/mapper_1/sensor_fov` | `visualization_msgs/Marker` | Transparent cyan inspection footprint cone (1.8m radius) sweeping the hull plate |
| `/{robot_id}/path_marker` | `visualization_msgs/Marker` | `LINE_STRIP` showing traversed trajectory trails |
| `/{robot_id}/goal_marker` | `visualization_msgs/Marker` | 3D `ARROW` indicating active target waypoint |
| `/geofence_status` | `std_msgs/String` | Boundary containment status and `out_of_bounds_count` |

### Defect Lifecycle State Progression
Every defect strictly transitions through 5 observable states:
1. **`UNDETECTED`**: Small, faint grey marker on hull surface prior to inspection.
2. **`DETECTED`**: Bright yellow marker when mapper sensor FOV cone covers the defect.
3. **`QUEUED`**: Industrial orange marker once utility scorer ingests and ranks the defect.
4. **`CLEANING`**: Vivid red pulsing marker with dual green/red progress bar during active cavitation cleaning.
5. **`VERIFIED_CLEAN`**: High-visibility green marker confirmed by mapper re-scan verification.

---

## 4. Quick Start & Execution

### 4.1 Build Workspace

```bash
source /opt/ros/lyrical/setup.bash
cd ws
colcon build --symlink-install
source install/setup.bash
```

### 4.2 Run Interactive Visual Simulation (Gazebo + RViz2 GUI)

```bash
./run_demo.sh
```

Opens RViz2 pre-configured with `RobotModel` displays for hull and robots, TF names hidden, readable marker scales, and camera centered on the entire 20m × 6m workspace.

### 4.3 Run Headless Simulation with 3-Stage Visual Snapshot Capture

```bash
./run_demo.sh --headless --duration=20
```

Captures 3 stages of the cleaning mission automatically into `docs/`:
- **Start:** [`docs/screenshot_start.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/screenshot_start.png) (Mapper sweeps hull & detects defects)
- **Mid-Cleaning:** [`docs/screenshot_mid_cleaning.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/screenshot_mid_cleaning.png) (Cleaner dispatched by utility score, active cavitation tool)
- **End:** [`docs/screenshot_end.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/screenshot_end.png) (Closed-loop verification confirmed `VERIFIED_CLEAN`)
- **Continuous:** [`docs/simulation_screenshot.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/simulation_screenshot.png)

---

## 5. Benchmarks & Evaluation Results

Automated benchmark evaluation is executed across 6 scenarios × 5 seeds (60 independent missions) comparing HullSentinal against a conventional naive FIFO baseline:

```bash
python3 ws/src/hull_benchmarks/hull_benchmarks/benchmark_runner.py
```

### Summary Results Table (Mean over 5 Seeds)

| Scenario | System Architecture | Time (s) | Order Correctness | OOB Violations | Detected / Cleaned | Risk Corr ($r_s$) | Catch Rate | Re-queues | Collisions |
|---|---|---|---|---|---|---|---|---|---|
| `nominal` | **HullSentinal (Ours)** | **78.2 ± 0.5** | **94.6%** | **0** | 9 / 9 | **+0.94** | 0.00 | 0.0 | 0 |
| `nominal` | Naive FIFO Baseline | 66.9 ± 2.7 | 59.9% | **0** | 9 / 9 | +0.17 | 0.00 | 0.0 | 0 |
| `swarm_scaling` | **HullSentinal (Ours)** | **76.4 ± 2.1** | **76.9%** | **0** | 18 / 18 | **+0.56** | 0.00 | 0.0 | 0 |
| `swarm_scaling` | Naive FIFO Baseline | 59.8 ± 1.8 | 55.5% | **0** | 18 / 18 | +0.09 | 0.00 | 0.0 | 0 |
| `obstacles` | **HullSentinal (Ours)** | **94.0 ± 0.5** | **94.6%** | **0** | 9 / 9 | **+0.94** | 0.00 | 0.0 | 0 |
| `high_sensor_noise` | **HullSentinal (Ours)** | **82.2 ± 4.8** | **94.5%** | **0** | 9 / 9 | **+0.94** | 0.00 | 0.0 | 0 |
| `agent_failure` | **HullSentinal (Ours)** | **90.4 ± 6.4** | **94.6%** | **0** | 9 / 9 | **+0.94** | **0.35** | **4.6** | 0 |
| `agent_failure` | Naive FIFO Baseline | 66.9 ± 2.7 | 59.9% | **0** | 9 / 9 | +0.17 | 0.00 (blind) | 0.0 (left dirty) | 0 |
| `marine_current` | **HullSentinal (Ours)** | **80.9 ± 0.5** | **94.6%** | **0** | 9 / 9 | **+0.94** | 0.00 | 0.0 | 0 |

Detailed statistics and figures:
- [`docs/BENCHMARK_REPORT.md`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/BENCHMARK_REPORT.md)
- [`docs/benchmark_results.csv`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/benchmark_results.csv)
- [`docs/plots/benchmark_order_correctness.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/plots/benchmark_order_correctness.png)
- [`docs/plots/benchmark_risk_correlation.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/plots/benchmark_risk_correlation.png)
- [`docs/plots/benchmark_completion_time.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/plots/benchmark_completion_time.png)

---

## 6. Running Unit & Integration Tests

```bash
# Run unit test suite (ORM, Scorer, Barrier, Decision Logic)
python3 ws/tests/test_core_logic.py

# Run Gate 2 automated defect lifecycle and geofencing verification test
python3 ws/tests/test_gate2_defect_lifecycle.py
```

All unit tests and automated lifecycle gates pass with 0 errors.
