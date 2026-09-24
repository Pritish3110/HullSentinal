# HullSentinal V2: Architectural Decisions Log

This document records key engineering decisions, tradeoffs, and rationales throughout the HullSentinal V2 upgrade.

---

## Decision 1: Single Centralized YAML Configuration (`hullsentinal_v2.yaml`)
- **Context:** V1 scattered parameters across Python default arguments, launch files, and individual scenario YAMLs, leading to magic numbers.
- **Decision:** Create `ws/src/hull_bringup/config/hullsentinal_v2.yaml` containing the complete configuration for hull geometry, lanes, obstacles, defects, thresholds, weights, and robot limits. Launch files and nodes ingest parameters from this single source of truth.
- **Rationale:** Ensures zero magic numbers and deterministic reproducibility across interactive demo runs, unit tests, and automated benchmarks.

## Decision 2: 24.0m × 8.0m Hull Scale with 3 Lanes & 6 Deterministic Obstacles
- **Context:** Section 3 requires a larger hull, dominant in RViz, with scaled spacing between lanes and defects, 4-8 static obstacles, and 12-18 defects.
- **Decision:**
  - Hull dimensions: 24.0m length × 8.0m height.
  - Lanes: 3 horizontal lanes centered at $Y = 2.0\text{m}, 4.0\text{m}, 6.0\text{m}$ with 1.8m lane width and 0.8m margin.
  - Obstacles: 6 deterministic rectangular obstacles placed strategically to force local detours on shortest paths without blocking defect access or completely closing a lane.
  - Defects: 14 defects (4 cracks, 5 rust, 5 biofouling) spread across all 3 lanes with non-colliding labels.
- **Rationale:** Satisfies all requirements with visual clarity and predictable path planning challenges.

## Decision 3: Single-Owner Defect Visualization Architecture
- **Context:** Section 12A highlights a critical defect label overlapping issue in V1 where multiple nodes published text markers with differing IDs.
- **Decision:** `MarkerPublisher` in `hull_bringup` is designated the strict, sole owner of all defect markers (`/defects`). All other nodes (sim, belief map, coordinator, tool controller) communicate solely via ROS topics (`/belief_updates`, `/defect_detections`). Marker IDs use a deterministic integer map indexed by defect ID, and update in place with `action = ADD`.
- **Rationale:** Eliminates ghost markers, duplicate labels, and text clobbering in RViz.

## Decision 4: Dedicated Grid Planner & 8-Connected A* Engine
- **Context:** Section 8 requires 2D occupancy grid mapping of the hull with inflated obstacles and 8-connected A* search without corner cutting.
- **Decision:** Implement a modular `GridPlanner` class in `hull_control` that builds a discrete grid (e.g. 0.2m resolution) from hull boundary and static obstacles, applies distance transform inflation ($0.55$m), and computes optimal paths with octile heuristics and proximity penalty fields.
- **Rationale:** Shared cleanly between Cleaner navigation and Mapper local obstacle detours, avoiding code duplication.

## Decision 5: Dedicated `LaneCoordinatorNode`
- **Context:** Section 6 mandates an explicit coordination policy rather than scattered if-statements across nodes.
- **Decision:** Create `ws/src/hull_swarm/hull_swarm/lane_coordinator_node.py` which tracks mapper progress, releases lanes when mapper transitions, marks tasks eligible in the belief map, and provides emergency eligibility overrides.
- **Rationale:** Decouples synchronization logic from low-level motion controllers and provides clean event tracing.
