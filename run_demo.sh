#!/usr/bin/env bash
# =============================================================================
# HullSentinal — Visual Simulation & Demo Runner
# Usage:
#   ./run_demo.sh             (Launches interactive full stack + RViz2 GUI)
#   ./run_demo.sh --headless  (Launches headless with visual snapshot capture)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ws"

echo "================================================================="
echo " HullSentinal: Cognitive Multi-Robot Ship Hull Cleaning Demo"
echo "================================================================="

# 1. Environment Activation
source /opt/ros/lyrical/setup.bash
if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
    source "${WS_DIR}/install/setup.bash"
else
    echo "[ERROR] Workspace not built. Run 'colcon build --symlink-install' in ./ws first."
    exit 1
fi

# 2. Parse arguments
GUI_MODE="true"
DURATION=0

for arg in "$@"; do
    case $arg in
        --headless)
            GUI_MODE="false"
            shift
            ;;
        --duration=*)
            DURATION="${arg#*=}"
            shift
            ;;
    esac
done

echo "[HullSentinal Demo] GUI Mode: ${GUI_MODE}"
echo "[HullSentinal Demo] Launching simulation stack..."

# 3. Clean up any previous dangling nodes
pkill -f "sim_engine_node|belief_map_node|utility_scorer|verification_monitor|crawler_controller|tool_controller|safety_barrier|geofence_node|marker_publisher|demo_snapshot" 2>/dev/null || true

# 4. Trap termination signals for clean shutdown
LAUNCH_PID=""

cleanup() {
    echo ""
    echo "[HullSentinal Demo] Shutting down simulation stack cleanly..."
    if [[ -n "${LAUNCH_PID}" ]]; then
        kill -INT "${LAUNCH_PID}" 2>/dev/null || true
        # Wait up to 3 seconds for graceful shutdown
        for i in {1..30}; do
            kill -0 "${LAUNCH_PID}" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    fi
    pkill -f "sim_engine_node|belief_map_node|utility_scorer|verification_monitor|crawler_controller|tool_controller|safety_barrier|geofence_node|marker_publisher|demo_snapshot" 2>/dev/null || true
    echo "[HullSentinal Demo] Shutdown complete."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 5. Launch ROS 2 Stack
if [[ "$GUI_MODE" == "false" ]]; then
    echo "[HullSentinal Demo] Running in headless mode (capturing visual snapshot to docs/simulation_screenshot.png)..."
    ros2 launch hull_bringup hull_full.launch.py gui:=false &
    LAUNCH_PID=$!

    # Wait for snapshot generation or specified duration
    WAIT_TIME=${DURATION:-10}
    echo "[HullSentinal Demo] Running for ${WAIT_TIME} seconds..."
    sleep "${WAIT_TIME}"

    if [[ -f "${SCRIPT_DIR}/docs/simulation_screenshot.png" ]]; then
        echo "[HullSentinal Demo] SUCCESS: Visual simulation screenshot captured at: docs/simulation_screenshot.png"
    fi
else
    echo "[HullSentinal Demo] Launching full interactive simulation with RViz2..."
    ros2 launch hull_bringup hull_full.launch.py gui:=true &
    LAUNCH_PID=$!
    wait "${LAUNCH_PID}"
fi
