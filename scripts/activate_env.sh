#!/usr/bin/env bash
# =============================================================================
# HullSentinal Project — Environment Activation Script
# Usage: source scripts/activate_env.sh   (must be sourced, NOT executed)
#
# Architecture (correct):
#   - System Python 3.14  ← runs all ROS2 nodes (rclpy, cv2, open3d, etc.)
#   - usv_swarm_env       ← use ONLY for standalone scripts (LangChain, PyTorch,
#                            analytics) that do NOT need rclpy
#
# NEVER mix conda Python 3.11 with sourced ROS2 setup.bash — the rclpy C
# extensions are compiled for Python 3.14 only.
# =============================================================================

# 1. Source system ROS2 Lyrical (sets AMENT_PREFIX_PATH, PATH, PYTHONPATH)
source /opt/ros/lyrical/setup.bash

# 2. If a local workspace install/overlay exists, source it on top
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source "${WS_ROOT}/install/setup.bash"
    echo "[HullSentinal] Workspace overlay sourced: ${WS_ROOT}/install/setup.bash"
fi

# 3. Sanity check
echo ""
echo "=== HullSentinal Environment Active ==="
echo "  Python    : $(python3 --version)"
echo "  ROS2      : $(ros2 --version 2>/dev/null || echo 'check PATH')"
echo ""
python3 -c "
import rclpy, cv2, open3d, matplotlib, numpy, sqlalchemy, dotenv
print('  rclpy      ✓')
print('  cv2        ✓  ' + cv2.__version__)
print('  open3d     ✓  ' + open3d.__version__)
print('  matplotlib ✓  ' + matplotlib.__version__)
print('  numpy      ✓  ' + numpy.__version__)
print('  sqlalchemy ✓  ' + sqlalchemy.__version__)
print('  python-dotenv ✓')
print()
print('  All imports OK — ready to develop.')
" 2>&1 || echo "  [ERROR] One or more imports failed."
echo "======================================="

# 4. Reminder about conda
echo ""
echo "  NOTE: Do NOT 'conda activate usv_swarm_env' in this shell."
echo "        ROS2 nodes use system Python 3.14 (already active)."
echo "        Use usv_swarm_env ONLY in a SEPARATE terminal for"
echo "        analytics/ML scripts that do NOT import rclpy."
echo ""
