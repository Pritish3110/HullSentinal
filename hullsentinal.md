# HullSentinal — Belief-Based Task Planning for Multi-Robot Ship Hull Cleaning

**A Cognitive Robotics Project — Simulation-Based Study**

| Field | Value |
|---|---|
| **Student 1** | Pritish Bhatasana — Reg. No. 23BRS1035 |
| **Student 2** | Siddharth Seth — Reg. No. 23BRS1038 |
| **Domain** | Cognitive Robotics |
| **Approach** | Simulation-first (Gazebo + ROS2 Lyrical) |
| **Workspace** | `~/ros2_ws/hullsentinal_ws` |
| **Conda Env** | `usv_swarm_env` (ROS2 Lyrical already installed) |

---

> **AGENT REFERENCE POLICY — READ FIRST**
>
> - All API keys, credentials, and secrets are stored in the `.env` file in the **project root directory**.
> - The agent **MUST reference API keys by their variable name only** (e.g., `OPENAI_API_KEY`, `ROBOFLOW_API_KEY`).
> - The agent **MUST NEVER open, read, write, inspect, or print the contents of `.env`** under any circumstances.
> - When code needs an API key, use `os.environ.get("KEY_NAME")` or `python-dotenv` loaded at app startup — never hardcode values.
> - If a key is missing at runtime, raise a descriptive `EnvironmentError` and stop — do not silently continue.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Environment & Repository Setup](#2-environment--repository-setup)
3. [Tech Stack & Tools](#3-tech-stack--tools)
4. [System Architecture](#4-system-architecture)
5. [ROS2 Package Structure](#5-ros2-package-structure)
6. [Component Specifications](#6-component-specifications)
7. [ROS2 Interfaces (Topics / Services / Actions)](#7-ros2-interfaces-topics--services--actions)
8. [Development Phases](#8-development-phases)
9. [Features Checklist](#9-features-checklist)
10. [Evaluation Plan](#10-evaluation-plan)
11. [Team & Roles](#11-team--roles)
12. [Known Risks & Mitigations](#12-known-risks--mitigations)
13. [Key References](#13-key-references)

---

## 1. Project Overview

Ship hulls accumulate biofouling (algae, barnacles), corrosion, and cracks over time, increasing fuel drag and risking structural damage. Manual underwater inspection and cleaning is slow, costly, and hazardous for divers.

This project builds a **fully independent, simulation-based system** using ROS2 Lyrical + Gazebo (Gz Sim). The novelty is placed in the **cognition layer**: a belief-driven, severity-aware task prioritization system with **closed-loop verification** — the one part of the pipeline not dependent on unavailable hardware and, per the literature review, not addressed by existing work in this domain.

### 1.1 Problem Statement

> Given a set of detected hull defects (crack, rust, biofouling) with varying severity and detection confidence, how should a two-robot system decide **what to clean first**, **in what order**, and **how to confirm the job was actually done** — without relying on real hardware, a fixed schedule, or blind task-completion assumptions?

### 1.2 Novelty Statement

No paper identified in the literature review (Section 13) combines all three of the following in one system:

1. Carrying **defect severity and detection confidence** into the task representation itself (a "belief").
2. Prioritizing cleaning tasks via an explicit **utility function**, not just travel-distance bidding or a static task list.
3. **Closing the loop** with post-cleaning re-verification — updating belief and re-queuing incomplete work rather than marking a task done once visited.

---

## 2. Environment & Repository Setup

### 2.1 Conda Environment

**Use existing environment: `usv_swarm_env`**

This environment already has:
- ROS2 Lyrical full stack (`rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `pcl_msgs`, `cv_bridge`, `tf2_ros`, `action_msgs`, `visualization_msgs`, etc.)
- NumPy, SciPy, SQLAlchemy, LangChain stack, PyTorch

**Still needs to be installed (pip inside `usv_swarm_env`):**
- `opencv-python` — defect detection heuristics
- `open3d` — point cloud handling
- `matplotlib` — logging/analytics plots
- `python-dotenv` — `.env` key loading at startup

Do **not** create a new conda environment. Activate with:

```bash
conda activate usv_swarm_env
source /opt/ros/lyrical/setup.bash
```

### 2.2 System-Level Packages (already installed)

Confirmed present on system:

| Package | Version |
|---|---|
| ROS2 Lyrical (full desktop) | Lyrical |
| `ros-lyrical-rviz2` | 15.2.5 |
| `ros-lyrical-cv-bridge` | 4.1.0 |
| `ros-lyrical-pcl-ros` | 2.10.0 |
| `ros-lyrical-pcl-conversions` | 2.10.0 |
| `ros-lyrical-pcl-msgs` | 1.0.0 |
| `ros-lyrical-robot-state-publisher` | 3.5.5 |
| `ros-lyrical-gz-cmake-vendor` | 0.4.5 |
| `ros-lyrical-gz-math-vendor` | 0.4.4 |

**Still missing (must install via apt):**
- `ros-lyrical-gz-ros2-control` or `ros-lyrical-ros-gz-sim` (Gz Sim bridge)
- `ros-lyrical-nav2-*` (navigation stack — for cleaning robot point-to-point motion)
- `ros-lyrical-slam-toolbox` (stretch goal)
- `gz-harmonic` or `gz-ionic` (Gazebo itself — must confirm which version pairs with Lyrical)

### 2.3 Repository Structure

```
hullsentinal_ws/
├── .env                          # API keys (NEVER read/write by agent)
├── .gitignore                    # include .env
├── README.md
├── install_deps.sh               # one-shot apt + pip installer script
├── src/
│   ├── hull_interfaces/          # Custom ROS2 msg/srv/action definitions
│   ├── hull_perception/          # Mapping robot: camera + defect detection node
│   ├── hull_cognition/           # Belief map, utility scorer, task queue manager
│   ├── hull_verification/        # Verification monitor node
│   ├── hull_cleaner/             # Cleaning robot node + state machine
│   ├── hull_simulation/          # Gazebo world, URDF/SDF models, launch files
│   └── hull_viz/                 # RViz2 config + marker publisher
├── config/
│   ├── utility_weights.yaml      # Hand-tuned scorer weights (documented)
│   └── sim_params.yaml           # Simulation parameters
├── logs/                         # Mission logs (JSON), auto-created at runtime
├── scripts/
│   ├── run_mission.sh            # Launch full simulation
│   └── run_baseline.sh           # Launch naive baseline for comparison
└── docs/
    ├── hullsentinal.md           # THIS FILE — master reference
    └── literature_review.md
```

---

## 3. Tech Stack & Tools

### 3.1 Core Platform

| Component | Choice | Notes |
|---|---|---|
| OS | Ubuntu 26.04 LTS | Development machine's actual OS |
| Middleware | ROS2 Lyrical (Lyrical Luth) | Latest ROS2 distro; released May 2026 |
| Simulator | Gazebo Harmonic (Gz Sim 8) | ROS2 Lyrical compatible; Gazebo Classic is EOL |
| Build system | colcon + ament_cmake / ament_python | Standard ROS2 workspace tooling |
| Python | 3.12 (inside `usv_swarm_env`) | Preferred for cognition layer |
| C++ | C++17 (optional, performance-critical nodes only) | Only if Python latency is a bottleneck |

### 3.2 Perception (Simulated)

| Component | Choice | Notes |
|---|---|---|
| Sensor | Simulated RGB-D camera (Gazebo plugin) | Publishes `sensor_msgs/Image` + `sensor_msgs/PointCloud2` |
| Defect detection | OpenCV color/contour thresholding on synthetic textured patches | Swappable later for YOLOv8 / U-Net |
| Point cloud | `pcl_ros` + Open3D (Python) | Accumulation and basic filtering |
| Noise injection | Gaussian noise on detection confidence | Makes verification loop non-trivial in demos |

### 3.3 Localization & Mapping

| Component | Choice | Notes |
|---|---|---|
| Primary | Gazebo ground-truth pose (`/model_states`) | Removes localization as a variable while cognition layer is developed |
| Stretch | `slam_toolbox` | ROS2-native 2D/2.5D SLAM |

### 3.4 Cognition Layer (Core Contribution)

| Component | Choice | Notes |
|---|---|---|
| Belief representation | Custom Python dataclass + SQLite (via SQLAlchemy) | Persistent across a session; queryable by defect_id |
| Utility scoring | `utility = severity × risk_weight × confidence − α × distance_cost` | Transparent, explainable; weights in `config/utility_weights.yaml` |
| Task queue | `heapq`-based priority queue + ROS2 publisher | Publishes ordered work list on `/task_queue` |
| Verification monitor | Re-scan comparison (contour area diff + confidence delta) | Triggers belief update + re-queue on incomplete cleaning |

### 3.5 Inter-Robot Communication

| Component | Choice |
|---|---|
| Transport | ROS2 topics + services + actions (DDS / FastDDS) |
| Detection publish | `/defect_detections` (topic, custom msg) |
| Task queue publish | `/task_queue` (topic, custom msg array) |
| Completion handshake | `/task_complete` (ROS2 service) |
| Re-scan request | `/rescan_request` (ROS2 service) |

### 3.6 Visualization & Tooling

| Component | Choice | Notes |
|---|---|---|
| Visualization | RViz2 | Robot pose, path, point cloud, defect markers with status color |
| Logging | JSON files in `logs/` + `rosbag2` (optional) | Structured mission logs for evaluation |
| Version control | Git / GitHub | Both team members push to shared repo |
| Documentation | This file (`hullsentinal.md`) | Always the master reference |

---

## 4. System Architecture

### 4.1 High-Level Architecture

```
                    ┌───────────────────────────────────────────┐
                    │              COGNITION LAYER               │
                    │                                            │
  ┌──────────────┐  │  ┌───────────┐   ┌────────────────┐       │  ┌──────────────┐
  │  Mapping     │  │  │ Belief Map│──▶│ Utility Scorer │       │  │  Cleaning    │
  │  Robot Node  │──┼─▶│(type, sev,│   │(severity×risk  │       │  │  Robot Node  │
  │              │  │  │ conf, pos)│   │  ×conf − cost) │       │  │              │
  │ PointCloud + │  │  └───────────┘   └───────┬────────┘       │  │ Navigates,   │
  │ defect flags │  │        ▲                 ▼                │  │ cleans,      │
  └──────────────┘  │        │         ┌────────────────┐       │  │ reports done │
         ▲          │        │         │ Task Queue     │───────┼─▶└──────┬───────┘
         │          │        │         │ Manager        │       │         │
         │          │        │         └────────────────┘       │         │
         │          │        │                                   │         ▼
         │          │  ┌─────┴──────────┐                       │  ┌──────────────┐
         └──────────┼──│ Verification   │◀──────────────────────┼──│/task_complete│
    (re-scan on     │  │ Monitor        │     re-scan result     │  │  (service)   │
     request)       │  └────────────────┘                       │  └──────────────┘
                    └───────────────────────────────────────────┘
```

### 4.2 Node Graph

```
[mapping_robot_node]
    → publishes → /defect_detections  (hull_interfaces/msg/DefectDetection)
    → publishes → /scan/points        (sensor_msgs/PointCloud2)
    ← service   → /rescan_request     (hull_interfaces/srv/RescanRequest)

[belief_map_node]         (part of hull_cognition)
    ← subscribes → /defect_detections
    → internal   → SQLite DB (defect store)
    → triggers   → utility_scorer on update

[utility_scorer_node]     (part of hull_cognition)
    ← triggered by belief map update
    → outputs scored list to task_queue_manager

[task_queue_manager_node] (part of hull_cognition)
    → publishes → /task_queue         (hull_interfaces/msg/TaskQueue)

[cleaning_robot_node]
    ← subscribes → /task_queue
    → navigates  → Nav2 / simple point-to-point controller
    → service    → /task_complete     (hull_interfaces/srv/TaskComplete)

[verification_monitor_node]
    ← service server → /task_complete
    → service call   → /rescan_request
    → updates        → belief_map_node (via service or topic)

[rviz_marker_publisher]   (part of hull_viz)
    ← subscribes → /defect_detections, /task_queue
    → publishes  → /visualization_marker_array
```

### 4.3 Data Flow (Step-by-Step)

1. Mapping robot traverses hull mesh → publishes `PointCloud2` + `DefectDetection` messages
2. Belief map node ingests detections → creates/updates defect records (SQLite)
3. Utility scorer re-ranks all `pending` / `re-queued` defects
4. Task queue manager publishes ordered `TaskQueue` message
5. Cleaning robot pops top task → navigates → executes simulated clean → calls `/task_complete`
6. Verification monitor requests re-scan → compares pre/post detection → updates belief
7. If defect persists: re-queue with updated severity/confidence; if cleared: mark `done`
8. Loop until queue empty or mission timer expires

### 4.4 Defect Belief State Machine

```
        [detected]
            │
            ▼
        PENDING ──── (scored + queued) ──▶ QUEUED
                                              │
                                     (cleaner pulls) ──▶ IN_PROGRESS
                                                              │
                                                    (reports done) ──▶ VERIFYING
                                                                           │
                                              ┌──────────────┬────────────┘
                                              ▼              ▼
                                          DONE          RE_QUEUED
                                                    (back to QUEUED
                                                     with updated score)
```

---

## 5. ROS2 Package Structure

### 5.1 `hull_interfaces` — Custom Messages / Services

**Messages:**

```
msg/DefectDetection.msg
    string defect_id        # UUID
    string defect_type      # "crack" | "rust" | "biofouling"
    float32 severity        # 0.0 – 1.0
    float32 confidence      # 0.0 – 1.0
    geometry_msgs/Point position   # 3D position on hull
    builtin_interfaces/Time stamp

msg/TaskItem.msg
    string defect_id
    float32 utility_score
    geometry_msgs/Point position
    string defect_type

msg/TaskQueue.msg
    TaskItem[] tasks        # Ordered highest-priority first
    builtin_interfaces/Time generated_at

msg/BeliefUpdate.msg
    string defect_id
    string new_status       # matches state machine states
    float32 new_severity
    float32 new_confidence
```

**Services:**

```
srv/TaskComplete.srv
    string defect_id
    bool reported_success
    ---
    bool verification_triggered

srv/RescanRequest.srv
    string defect_id
    geometry_msgs/Point position
    float32 scan_radius
    ---
    DefectDetection[] detections  # empty if clean
```

### 5.2 Package Dependency Map

```
hull_simulation   →  (no internal deps)
hull_interfaces   →  (no internal deps; depends on std ROS2 msgs)
hull_perception   →  hull_interfaces
hull_cognition    →  hull_interfaces, hull_perception (msg types)
hull_verification →  hull_interfaces, hull_cognition
hull_cleaner      →  hull_interfaces, hull_cognition
hull_viz          →  hull_interfaces
```

---

## 6. Component Specifications

### 6.1 Belief Map (`hull_cognition/belief_map_node.py`)

**Schema (SQLite via SQLAlchemy):**

```python
class DefectBelief(Base):
    defect_id: str (PK, UUID)
    defect_type: str           # "crack" | "rust" | "biofouling"
    severity: float            # 0.0 – 1.0
    confidence: float          # 0.0 – 1.0
    position_x: float
    position_y: float
    position_z: float
    status: str                # PENDING | QUEUED | IN_PROGRESS | VERIFYING | DONE | RE_QUEUED
    detection_time: datetime
    last_updated: datetime
    rescan_count: int          # how many times verification has triggered
```

**Responsibilities:**
- Subscribe to `/defect_detections`, upsert records
- Expose internal query API to Utility Scorer
- Emit `/belief_updates` topic on any status/score change

### 6.2 Utility Scorer (`hull_cognition/utility_scorer.py`)

**Formula:**

```
utility(d) = (severity(d) × risk_weight(type(d)) × confidence(d))
             − α × euclidean_distance(cleaner_pos, position(d))
```

**Risk weights** (in `config/utility_weights.yaml`, documented reasoning):

```yaml
risk_weights:
  crack: 1.0        # structural risk — highest
  rust: 0.7         # degradation risk — medium
  biofouling: 0.4   # drag penalty — lower structural risk
distance_cost_alpha: 0.1   # scale factor — tunable
```

**Responsibilities:**
- Re-score all non-DONE defects on each belief update
- Pass scored list to Task Queue Manager
- Subscribe to `/cleaner_pose` for distance calculation

### 6.3 Task Queue Manager (`hull_cognition/task_queue_manager_node.py`)

- Uses Python `heapq` (max-heap via negated utility)
- Publishes `/task_queue` at configurable rate (default 1 Hz) and on every update
- Pops a task when the cleaning robot sends a pull request (via service or topic)

### 6.4 Mapping Robot Node (`hull_perception/mapping_robot_node.py`)

- Reads Gazebo ground-truth pose via `/model_states` or TF
- Moves along a predefined sweep path on the hull (configurable waypoints in `sim_params.yaml`)
- Runs OpenCV color/contour detection on camera frames
- Injects **configurable Gaussian noise** on severity/confidence (to make verification meaningful)
- Handles `/rescan_request` — revisits a specific 3D position and re-publishes detection result

**Defect detection heuristic:**
```
HSV thresholding:
  crack      → dark/grey tones, elongated contour aspect ratio > 3
  rust       → hue 0–15 (red-orange)
  biofouling → hue 70–140 (green/teal)
Severity ≈ normalized contour area
Confidence ≈ (1 − noise_std) × detection_quality
```

### 6.5 Cleaning Robot Node (`hull_cleaner/cleaning_robot_node.py`)

**State machine:**
```
IDLE → NAVIGATING → AT_SITE → CLEANING → REPORTING → IDLE
```

- Navigation: Nav2 `NavigateToPose` action client (primary), or simple point-to-point if Nav2 not available
- Cleaning simulation: timer-based (`sim_params.yaml` configures clean duration per defect type)
- On completion: call `/task_complete` service
- Publishes pose on `/cleaner_pose` at 5 Hz for utility scorer distance updates

### 6.6 Verification Monitor (`hull_verification/verification_monitor_node.py`)

- Service server for `/task_complete`
- On call: request re-scan via `/rescan_request`
- Compare result:
  - If **no detection** in re-scan → mark `DONE` in belief map
  - If **detection persists** → update `severity` and `confidence` from re-scan, set status `RE_QUEUED`
  - If **severity increased** (worsened) → also log a warning + escalate utility score

### 6.7 Simulation World (`hull_simulation/`)

- SDF world file: static ship hull box mesh with synthetic texture patches
- Defect patches: colored material overlays (configurable count, type, position via `sim_params.yaml`)
- Robot models: URDF/SDF for mapper (small, wheeled/magnetic) and cleaner (larger)
- Gazebo RGB-D camera plugin on mapper robot
- Launch file: `hull_full.launch.py` (brings up Gazebo + all nodes + RViz2)

---

## 7. ROS2 Interfaces (Topics / Services / Actions)

| Name | Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/defect_detections` | `hull_interfaces/msg/DefectDetection` | `mapping_robot_node` | `belief_map_node`, `rviz_marker_publisher` |
| `/scan/points` | `sensor_msgs/PointCloud2` | `mapping_robot_node` | RViz2 |
| `/task_queue` | `hull_interfaces/msg/TaskQueue` | `task_queue_manager_node` | `cleaning_robot_node`, RViz2 |
| `/cleaner_pose` | `geometry_msgs/PoseStamped` | `cleaning_robot_node` | `utility_scorer` |
| `/belief_updates` | `hull_interfaces/msg/BeliefUpdate` | `belief_map_node` | `verification_monitor_node`, RViz2 |
| `/visualization_marker_array` | `visualization_msgs/MarkerArray` | `rviz_marker_publisher` | RViz2 |
| `/task_complete` | `hull_interfaces/srv/TaskComplete` | (client) `cleaning_robot_node` | (server) `verification_monitor_node` |
| `/rescan_request` | `hull_interfaces/srv/RescanRequest` | (client) `verification_monitor_node` | (server) `mapping_robot_node` |

---

## 8. Development Phases

### Phase 1 — Environment & Scaffold (Week 1)

- [ ] Activate `usv_swarm_env`, source ROS2 Lyrical, verify `rclpy` works
- [ ] Install missing pip packages (`opencv-python`, `open3d`, `matplotlib`, `python-dotenv`)
- [ ] Install missing apt packages (Gz Sim, Nav2, see install commands in Section 2)
- [ ] Create `hullsentinal_ws` with colcon structure
- [ ] Create `hull_interfaces` package; define all msgs/srvs; `colcon build`
- [ ] Verify custom interfaces build and are importable

### Phase 2 — Simulation World (Week 1–2)

- [ ] Create Gazebo world SDF with static hull mesh (simple box geometry)
- [ ] Add synthetic defect texture patches (color-coded materials)
- [ ] Create URDF for mapping robot (box chassis + RGB-D camera plugin)
- [ ] Create URDF for cleaning robot (box chassis + no sensor)
- [ ] Confirm both robots spawn and Gazebo camera publishes `sensor_msgs/Image`

### Phase 3 — Perception Node (Week 2)

- [ ] Implement `mapping_robot_node.py` with:
  - Sweep path traversal (waypoint following)
  - RGB-D subscription → OpenCV detection pipeline
  - `DefectDetection` message publish with Gaussian noise
  - `/rescan_request` service handler
- [ ] Test: run node, echo `/defect_detections`, confirm detections appear for colored patches

### Phase 4 — Cognition Layer (Week 2–3)

- [ ] Implement `belief_map_node.py` + SQLite schema (SQLAlchemy already in env)
- [ ] Implement `utility_scorer.py` with configurable weights YAML
- [ ] Implement `task_queue_manager_node.py` with heapq + `/task_queue` publish
- [ ] Unit test: inject synthetic detections, verify queue order matches utility formula
- [ ] Integration test: run perception + cognition nodes, verify belief map populates

### Phase 5 — Cleaning Robot & Verification (Week 3)

- [ ] Implement `cleaning_robot_node.py` with state machine + Nav2 client
- [ ] Implement `verification_monitor_node.py` with re-scan + belief update logic
- [ ] Wire up `/task_complete` ↔ `/rescan_request` ↔ belief update pipeline
- [ ] End-to-end test: full loop — detect → queue → clean → verify → re-queue (if partial)

### Phase 6 — Visualization & Evaluation (Week 4)

- [ ] Implement `rviz_marker_publisher.py` (defect markers colored by status + type)
- [ ] Configure RViz2 layout (point cloud, markers, robot models, TF)
- [ ] Implement mission logger (JSON logs in `logs/`)
- [ ] Implement naive baseline (FIFO discovery order, no re-verification) for comparison
- [ ] Run 3+ simulated missions, record metrics (see Section 10)
- [ ] Record demo video; write final report; prepare presentation

---

## 9. Features Checklist

### MVP (Must Have)

- [ ] Gazebo world with hull mesh + synthetic color-coded defects
- [ ] Simulated RGB-D camera → OpenCV detection → `DefectDetection` publish
- [ ] Belief map with SQLite persistence + status state machine
- [ ] Utility function: `severity × risk_weight × confidence − α × distance_cost`
- [ ] Task queue ordered by utility, published over ROS2
- [ ] Cleaning robot: navigate to defect + simulated clean action + completion report
- [ ] Verification loop: re-scan after cleaning, belief update, re-queue if incomplete
- [ ] RViz2: defect markers (color by status), robot poses, point cloud, task queue order
- [ ] Gaussian noise on detections to make verification non-trivial
- [ ] JSON mission logs + evaluation metrics output

### Stretch Goals (Time-Permitting)

- [ ] `slam_toolbox` integration (replace ground-truth pose)
- [ ] Confidence decay over time (stale detections lose weight)
- [ ] Multiple simultaneous cleaning robots with task-splitting
- [ ] Analytics dashboard (`matplotlib`) comparing prioritized vs. naive order
- [ ] YOLOv8 swap-in for color heuristic detector
- [ ] Cleaning robot "task infeasible" failure handling + replanning

### Explicitly Out of Scope

- Real ZED2i hardware integration
- Reproducing RTAB-Map / hull-surface path planning algorithms
- Deep-learning defect classification as the project's research contribution
- Physical multi-robot deployment

---

## 10. Evaluation Plan

| Metric | What it Measures | How to Measure |
|---|---|---|
| Task completion order vs. utility ranking | Scorer drives cleaning order as intended | Log utility scores at dispatch time; compare to arrival order |
| Verification catch rate | Re-scanning catches incomplete/failed cleans | Count re-queue events / total tasks; tune noise level |
| Time-to-clean-all: prioritized vs. naive | Practical benefit of prioritization | Mission timer; naive baseline: FIFO, no re-verify |
| Re-queue count per mission | Closed-loop verification is functioning | Count `RE_QUEUED` state transitions in SQLite |
| Utility score evolution | Belief updates propagate correctly | Plot utility scores over mission time per defect |

**Baseline for comparison:** A naive node that dispatches defects in discovery order (FIFO), marks tasks done immediately after cleaning (no verification), and ignores severity/confidence.

---

## 11. Team & Roles

| Member | Primary Responsibilities |
|---|---|
| Pritish Bhatasana (23BRS1035) | Belief map + utility scoring; simulation world setup; `hull_interfaces` package; mission logger |
| Siddharth Seth (23BRS1038) | Verification monitor + inter-robot ROS2 messaging; cleaning robot state machine; RViz2 visualization |

**Integration boundary:** `/task_queue` topic and `/task_complete` service — both members share responsibility for these interfaces.

---

## 12. Known Risks & Mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| Gz Sim ↔ ROS2 Lyrical plugin incompatibility | Medium | Test Gazebo spawn early in Phase 1; fallback: use `gazebo_ros_pkgs` bridge or mock sensor publisher |
| Nav2 not available / partially installed | Medium | Implement simple point-to-point fallback controller; Nav2 is optional for MVP |
| Noiseless synthetic detection makes verification trivial | High | Mandatory Gaussian noise injection on confidence (configurable std dev in `sim_params.yaml`) |
| Utility function weights not well-calibrated | Low | Document reasoning in `config/utility_weights.yaml`; present as design choices, not empirical results |
| SQLite write contention (multi-node access) | Low | Use SQLAlchemy sessions with appropriate isolation; or switch to in-memory dict if latency is critical |
| ROS2 Lyrical documentation sparse (new distro) | Medium | Fall back to Jazzy/Humble docs for API — ABI is compatible for most rclpy interfaces |

---

## 13. Key References

1. Su, H., Liu, S., Zhang, L., Chen, Y., Yang, C. — "Biofouling recognition and boundary tracking control for underwater cleaning robots." *Ocean Engineering*, Vol. 295 (2024), Art. 116707.
2. "Automated detection and classification of cracks in ship hull structures using deep learning." *Ocean Engineering* (2026).
3. "Detection of Cracks and Corrosion for Automated Vessels Visual Inspection" (LIACI dataset), EU FP7 MINOAS project.
4. "Design, perception, planning and control for underwater cleaning robotics: Progress, challenges and trends." *Ocean Engineering* review (2025).
5. "Algorithm for Determining the Position of a Ship Hull-Cleaning Robot." *Applied Sciences* 15(10), 5705 (2025).
6. Ruediger, M., Banerjee, A.G. — "Mapping-Guided Task Discovery and Allocation for Cooperative Robotic Inspection of Underwater Structures" (Map-TIDAL). arXiv:2602.02389 (2026).
7. "A Centralized Multi-Robot Task Allocation for Industrial Plant Inspection by Using A* and Genetic Algorithms."
8. "A Hierarchical LLM-Based Framework for Heterogeneous Multi-Robot Orchestration in High-Risk Energy Facility Maintenance" (2026).

*(Full comparison and links in `docs/literature_review.md`.)*
