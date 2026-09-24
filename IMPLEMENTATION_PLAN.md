# HullSentinal V2: Implementation Plan

Mapping of all V2 Cognitive Multi-Robot Upgrade requirements to specific packages and files.

---

## 1. Requirement to File Mapping

| Step / Requirement | Files to Modify / Create | Description of Changes |
|---|---|---|
| **Step 1: Inspect & Plan** | `ARCHITECTURE_NOTES.md`<br>`IMPLEMENTATION_PLAN.md`<br>`DECISIONS.md` | Complete architectural baseline audit, requirement mapping, and initial git commit. |
| **Step 2: Configuration** | `ws/src/hull_bringup/config/hullsentinal_v2.yaml`<br>`ws/src/hull_bringup/config/default_params.yaml` | Single unified parameter configuration without magic numbers (dimensions, lanes, sensor FOV, risk thresholds, utility weights, 14 defects, 6 obstacles, safety margins). |
| **Step 3: World & Environment** | `ws/src/hull_description/urdf/ship_hull.urdf.xacro`<br>`ws/src/hull_bringup/config/rviz_config.rviz` | Expand hull to 24m × 8m dominant surface plate, deep ocean backplane (32m × 14m), perimeter rub rails, deterministic static obstacles, lane reference guides. |
| **Step 4: Ground Truth vs Belief** | `ws/src/hull_msgs/msg/BeliefUpdate.msg`<br>`ws/src/hull_msgs/msg/TaskItem.msg`<br>`ws/src/hull_sim/hull_sim/sim_engine_node.py`<br>`ws/src/hull_swarm/hull_swarm/belief_map_node.py` | Ground truth in `sim_engine_node` with deterministic severity growth (one defect escalating past CRITICAL). `BeliefMapNode` maintains full defect schema and is the single shared state engine. |
| **Step 5: Mapper Serpentine Sweep** | `ws/src/hull_control/hull_control/mapper_controller.py`<br>`ws/src/hull_msgs/msg/MapperStatus.msg` | Deterministic serpentine coverage: Lane 1 (L->R), Lane 2 (R->L), Lane 3 (L->R). Publishes mission state (`MAPPING_LANE_1..N`, `MAPPING_COMPLETE`) and per-lane coverage %. Local obstacle detours. |
| **Step 6: Lane Coordinator Node** | `ws/src/hull_swarm/hull_swarm/lane_coordinator_node.py`<br>`ws/src/hull_msgs/msg/LaneCoordination.msg` | New node enforcing policy: cleaner cleans lanes $< N$. Emits events (`MAPPER_ENTERED_LANE_k`, `LANE_k_RELEASED`, `MAPPING_COMPLETE`). CRITICAL emergency immediate eligibility override. |
| **Step 7: Cleaner State Machine & Utility** | `ws/src/hull_control/hull_control/tool_controller.py`<br>`ws/src/hull_swarm/hull_swarm/utility_scorer.py` | Explicit enums & transition tables (`CleanerState`, `TaskState`). Utility incorporates real A* path cost, risk weights, confidence, age bonus, and emergency override. Structured logging. |
| **Step 8: 2D Occupancy Grid & A* Navigation** | `ws/src/hull_control/hull_control/grid_planner.py`<br>`ws/src/hull_control/hull_control/crawler_controller.py` | 2D hull occupancy grid with inflated obstacles ($0.55$m). 8-connected A* with octile heuristic and proximity penalty. Waypoint-by-waypoint path following and dynamic replanning. |
| **Step 9: Dynamic Safety & Conflict Avoidance** | `ws/src/hull_control/hull_control/safety_barrier.py` | Decision hierarchy: (1) robot safety (2) geofence (3) collision avoidance (4) CRITICAL emergency (5) normal task. Trajectory conflict prediction (time-to-conflict). Priority rules and deadlock sidestepping. |
| **Step 10: Emergency Interruption & Resume** | `ws/src/hull_control/hull_control/tool_controller.py` | Triggers when belief severity $\ge 0.85$. Safely stops tool, pushes context (task, path, waypoint, queue) onto priority stack, executes A* emergency remediation, restores context, re-ranks queue. |
| **Step 11: Closed-Loop Verification** | `ws/src/hull_swarm/hull_swarm/verification_monitor.py` | Ensures cleaning $\neq$ success. Closed-loop mapper rescan confirms zero residual. Incomplete cleans transition to `FAILED` or `REQUEUED`. |
| **Step 12: Single-Owner MarkerPublisher & HUD** | `ws/src/hull_bringup/hull_bringup/marker_publisher.py`<br>`ws/src/hull_bringup/config/rviz_config.rviz` | Single owner of defect visuals. Stable namespace + ID (`defect_label`, `defect_shape`). Pure function of belief state. ASCII text. Delete-all on init. Non-overlapping HUD panel. Centered camera. |
| **Step 13: Structured Logging** | All Python nodes in `ws/src/` | Structured tags: `[MAPPER]`, `[BELIEF]`, `[UTILITY]`, `[COORDINATION]`, `[CLEANER]`, `[PLANNER]`, `[SAFETY]`, `[EMERGENCY]`, `[VERIFICATION]`. |
| **Step 14: Demo Scenario Launch** | `ws/src/hull_bringup/launch/hull_full.launch.py`<br>`run_demo.sh` | Orchestrates complete pipeline with single YAML config, 3 lanes, 14 defects, 6 obstacles, and escalating crack. |
| **Step 15: Acceptance Testing & Benchmarks** | `ws/tests/test_v2_acceptance.py`<br>`ws/src/hull_benchmarks/hull_benchmarks/benchmark_runner.py` | Automated acceptance test suite validating all 8 functional gates. Benchmark suite updated with 7 new metrics across 60 runs. Screenshots captured into `docs/`. |
| **Step 16: Final Deliverables** | `CHANGES.md`<br>`README.md`<br>`DONE.md` | Comprehensive release documentation, state machine diagrams, architecture notes, and verification logs. |

---

## 2. Gate Verification Milestones

Every step must pass its gate with concrete evidence (logs, passing tests, or screenshots) before committing to git:
1. **Config & World Gate:** YAML loads cleanly; RViz displays 24m × 8m hull with 3 lanes and obstacles without warnings.
2. **Mapper Serpentine Gate:** Mapper sweeps Lane 1 $\to$ Lane 2 $\to$ Lane 3 in order, detouring around obstacles, publishing coverage %.
3. **Lane Coordinator Gate:** Cleaner stays in `WAITING_FOR_LANE` until Lane 1 is released. Zero lane eligibility violations.
4. **Planning & Safety Gate:** Cleaner plans A* around obstacles; minimum robot distance $\ge 0.6$m; zero collisions.
5. **Emergency Interruption Gate:** Escalating defect triggers interrupt mid-cleaning; saved to stack; emergency cleaned & verified; queue restored.
6. **Single-Owner Labels Gate:** Exactly 1 label per defect, stable IDs, ASCII text, no overlapping duplicates.
7. **Full Acceptance Gate:** `test_v2_acceptance.py` passes 100%; benchmark runner completes 60 runs; 6 screenshots captured into `docs/`.
