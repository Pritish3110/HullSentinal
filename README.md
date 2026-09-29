# HullSentinal V2: Cognitive Multi-Robot Ship Hull Inspection & Cleaning

**Cognitive Multi-Robot System — Closed-Loop Swarm Inspection, Prioritized Remediation & Lane Coordination**  
**OS:** Ubuntu 26.04 (Resolute) | **Middleware:** ROS 2 Lyrical | **Python:** 3.14  
**Workspace:** `./ws`

---

## 1. Project Overview

Ship hull degradation (biofouling, corrosion/rust, structural fatigue cracks) drastically increases hydrodynamic drag, incurs severe bunker fuel penalties, and poses grave structural failure risks. Conventional inspection and cleaning systems suffer from two fatal operational flaws:
1. **Blind Task Allocation:** Work is dispatched strictly by proximity or FIFO arrival order, ignoring defect severity and structural criticality.
2. **Open-Loop Assumption:** Tasks are marked "completed" simply upon reaching coordinates, with zero validation that the defect was actually remediated.

**HullSentinal V2** resolves this through a cognitive multi-robot architecture on a scaled **24m × 8m** marine steel hull manifold:
- **Centralized Configuration (`hullsentinal_v2.yaml`):** Zero magic numbers; all hull bounds, lanes, obstacles, defect models, and robot kinematics load from a unified YAML specification.
- **3-Lane Serpentine Inspection:** An agile mapper ROV performs continuous serpentine sweeps across 3 horizontal lanes ($Y = 2.0, 4.0, 6.0\text{m}$) executing local A* detours around static hazards.
- **Dedicated Lane Coordinator:** Cleaner crawlers are strictly constrained to remediate defects in completed lanes ($\text{cleaner\_lane} < \text{mapper\_lane}$), avoiding spatial interference and maintaining clear operational corridors.
- **Dynamic Defect Growth & Critical Preemption:** Defects undergo physical progression ($\Delta s = 0.015/\text{s}$). When a defect escalates past `CRITICAL` ($s \ge 0.85$), the cleaner immediately halts its current task, pushes its execution state onto a **Priority Context Stack**, executes emergency remediation, verifies cleanliness, and seamlessly resumes its interrupted task.
- **8-Connected A* Hazard Navigation:** 6 deterministic obstacles (sea chests, sacrificial anodes, discharge ports) are inflated by $0.55\text{m}$, ensuring collision-free paths with non-corner-cutting diagonal moves.
- **Closed-Loop Verification Monitor:** When cleaning completes, a re-scan request is dispatched. Residual defects are caught and rescheduled with elevated priority.
- **Predictive Dynamic Safety Barrier:** Continuous time-to-conflict trajectory evaluation preserves mapper right-of-way and executes lateral sidestep maneuvers to resolve deadlocks.
- **Strict Perimeter Geofence Containment:** Real-time enforcement within $[0.8, 23.2] \times [0.8, 7.2]$ ensures zero boundary breaches (`out_of_bounds_count == 0`, `lane_violation_count == 0`).

---

## 2. Workspace Package Structure (`./ws/src`)

| Package | Build Type | Description |
|---|---|---|
| [`hull_msgs`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_msgs) | `ament_cmake` | DDS interfaces: `DefectDetection`, `TaskItem`, `TaskQueue`, `BeliefUpdate`, `RobotStatus`, `MapperStatus`, `MissionStatus`, `TaskComplete.srv`, `RescanRequest.srv` |
| [`hull_description`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_description) | `ament_cmake` | URDF/Xacro models: Mapper ROV (cyan, sensor pod, dual LEDs), Cleaner crawler (orange, magnetic treads, lime cavitation brush), and 24m×8m steel hull plate with ocean backplane and safety perimeter rub rails |
| [`hull_sim`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_sim) | `ament_python` | High-fidelity multi-robot kinematic simulation, sensor FOV raycasting, defect degradation physics, dynamic growth models, and Gazebo world |
| [`hull_control`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_control) | `ament_python` | Serpentine mapper controller with obstacle detours, 8-connected A* grid planner, crawler controller with Priority Context Stack preemption, geofence monitor, and dynamic safety barrier |
| [`hull_swarm`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_swarm) | `ament_python` | Cognitive coordination layer: Lane Coordinator, SQLAlchemy SQLite Belief Map, dynamic Utility Scorer, and closed-loop Verification Monitor |
| [`hull_bringup`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_bringup) | `ament_python` | Master launch files (`hull_full.launch.py`), RViz2 configuration, single-owner Marker Publisher, and visual demo snapshot node |
| [`hull_benchmarks`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/ws/src/hull_benchmarks) | `ament_python` | Automated 60-mission benchmark runner across 6 scenarios × 5 seeds, naive FIFO baseline comparison, and statistical report generator |

---

## 3. High-Fidelity RViz2 Visualization & Topics

The single-owner marker pipeline (`marker_publisher.py`) manages all RViz displays with stable `(namespace, id)` indexing, startup `DELETEALL` sweeps, and in-place updates:

| Topic | Type | Visual Representation |
|---|---|---|
| `/mapper_1/robot_description` | `std_msgs/String` | 3D inspection ROV body (>0.65m) in bright cyan with camera sensor pod and dual LED illuminators |
| `/cleaner_1/robot_description` | `std_msgs/String` | 3D industrial crawler body (>0.70m) in industrial orange with magnetic crawler treads and lime green cavitation brush |
| `/hull/robot_description` | `std_msgs/String` | 24.0m × 8.0m marine steel hull plate, deep ocean water backplane, structural weld seams, and yellow safety rub rails |
| `/defects` | `visualization_msgs/MarkerArray` | Ground-truth defects with shapes by type (**cracks**=cubes, **rust**=discs, **biofouling**=spheres), scale proportional to severity, and text status labels |
| `/belief_map/heatmap` | `visualization_msgs/MarkerArray` | 2D hull surface heatmap showing per-cell belief probability and utility density |
| `/system_hud_marker` | `visualization_msgs/Marker` | Real-time HUD table at $(12.0, 7.5, 0.4)$ displaying active mapper state, lane authorization, queue count, and emergency status |
| `/mapper_1/sensor_fov` | `visualization_msgs/Marker` | Transparent cyan inspection footprint cone (1.8m radius) sweeping the hull plate |
| `/{robot_id}/path_marker` | `visualization_msgs/Marker` | `LINE_STRIP` showing traversed trajectory trails |
| `/{robot_id}/goal_marker` | `visualization_msgs/Marker` | 3D `ARROW` indicating active target waypoint |
| `/geofence_status` | `std_msgs/String` | Boundary containment status and `out_of_bounds_count` |

### Defect Lifecycle State Progression
Every defect strictly transitions through 5 observable states:
1. **`UNDETECTED`**: Small, faint grey marker on hull surface prior to inspection.
2. **`DETECTED`**: Bright yellow marker when mapper sensor FOV cone covers the defect.
3. **`QUEUED`**: Industrial orange marker once utility scorer ingests and ranks the defect.
4. **`CLEANING`**: Vivid red pulsing marker with dual progress bar during active cavitation cleaning.
5. **`VERIFIED_CLEAN`**: High-visibility green marker confirmed by closed-loop re-scan verification.

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

Opens RViz2 pre-configured with `RobotModel` displays for hull and robots, TF names hidden, readable marker scales, and camera centered on the entire 24m × 8m workspace.

### 4.3 Run Headless Simulation with Multi-Stage Visual Snapshot Capture

```bash
./run_demo.sh --headless --duration=30
```

Captures visual stages of the cleaning mission automatically into `docs/`:
- **Start:** [`docs/screenshot_start.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/screenshot_start.png) (Mapper begins Lane 1 sweep; defects undetected)
- **Lane Coordination:** [`docs/screenshot_lane_coordination.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/screenshot_lane_coordination.png) (Mapper completes Lane 1, releasing it for cleaner entry)
- **Mid-Cleaning:** [`docs/screenshot_mid_cleaning.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/screenshot_mid_cleaning.png) (Cleaner dispatched by A* utility, cavitation tool active)
- **Emergency Interruption:** [`docs/screenshot_emergency_interruption.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/screenshot_emergency_interruption.png) (Critical crack escalation preempts cleaner via context stack)
- **End:** [`docs/screenshot_end.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/screenshot_end.png) (Closed-loop verification confirmed `VERIFIED_CLEAN`)
- **Continuous:** [`docs/simulation_screenshot.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/simulation_screenshot.png)

---

## 5. Benchmarks & Evaluation Results

Automated benchmark evaluation is executed across 6 scenarios × 5 seeds (60 independent missions) comparing HullSentinal V2 against a conventional naive FIFO baseline:

```bash
python3 ws/src/hull_benchmarks/hull_benchmarks/benchmark_runner.py
```

### Summary Results Table (Mean over 5 Seeds)

| Scenario | System Architecture | Time (s) | Order Correctness | OOB Violations | Lane Violations | Detected / Cleaned | Risk Corr ($r_s$) | Catch Rate | Collisions |
|---|---|---|---|---|---|---|---|---|---|
| `nominal` | **HullSentinal V2 (Ours)** | **78.2 ± 0.5** | **94.6%** | **0** | **0** | 14 / 14 | **+0.94** | 0.00 | 0 |
| `nominal` | Naive FIFO Baseline | 66.9 ± 2.7 | 59.9% | **0** | **0** | 14 / 14 | +0.17 | 0.00 | 0 |
| `swarm_scaling` | **HullSentinal V2 (Ours)** | **76.4 ± 2.1** | **76.9%** | **0** | **0** | 14 / 14 | **+0.56** | 0.00 | 0 |
| `swarm_scaling` | Naive FIFO Baseline | 59.8 ± 1.8 | 55.5% | **0** | **0** | 14 / 14 | +0.09 | 0.00 | 0 |
| `obstacles` | **HullSentinal V2 (Ours)** | **94.0 ± 0.5** | **94.6%** | **0** | **0** | 14 / 14 | **+0.94** | 0.00 | 0 |
| `high_sensor_noise` | **HullSentinal V2 (Ours)** | **82.2 ± 4.8** | **94.5%** | **0** | **0** | 14 / 14 | **+0.94** | 0.00 | 0 |
| `agent_failure` | **HullSentinal V2 (Ours)** | **90.4 ± 6.4** | **94.6%** | **0** | **0** | 14 / 14 | **+0.94** | **0.35** | 0 |
| `agent_failure` | Naive FIFO Baseline | 66.9 ± 2.7 | 59.9% | **0** | **0** | 14 / 14 | +0.17 | 0.00 (blind) | 0 |
| `marine_current` | **HullSentinal V2 (Ours)** | **80.9 ± 0.5** | **94.6%** | **0** | **0** | 14 / 14 | **+0.94** | 0.00 | 0 |

Detailed statistics and figures:
- [`docs/BENCHMARK_REPORT.md`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/BENCHMARK_REPORT.md)
- [`docs/benchmark_results.csv`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/benchmark_results.csv)
- [`docs/plots/benchmark_order_correctness.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/plots/benchmark_order_correctness.png)
- [`docs/plots/benchmark_risk_correlation.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/plots/benchmark_risk_correlation.png)
- [`docs/plots/benchmark_completion_time.png`](file:///home/pritish_3110/Personal%20Files/Syllabus/Projects/HullSentinal/docs/plots/benchmark_completion_time.png)

---

## 6. Running Acceptance & Integration Tests

```bash
# Sourcing the workspace
source /opt/ros/lyrical/setup.bash
source ws/install/setup.bash

# Run full V2 acceptance test suite (8 gates validated)
python3 ws/tests/test_v2_acceptance.py

# Run core logic unit test suite
python3 ws/tests/test_core_logic.py
```

All 8 V2 acceptance gates and core unit tests pass with 0 errors:
- `test_gate1_configuration_and_world`: PASSED
- `test_gate2_grid_planner_and_astar_detours`: PASSED
- `test_gate3_serpentine_mapper_progression`: PASSED
- `test_gate4_lane_coordination_and_cleaner_gating`: PASSED
- `test_gate5_defect_growth_and_emergency_escalation`: PASSED
- `test_gate6_emergency_context_stack_preemption`: PASSED
- `test_gate7_single_owner_marker_indexing`: PASSED
- `test_gate8_geofence_and_boundary_containment`: PASSED
