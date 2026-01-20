#!/bin/bash
set -e  # Exit on error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
PROJECT_NAME="cine_gimbal_control"

echo "=============================================="
echo "   Cine Gimbal Control - Build Only           "
echo "=============================================="

# 1. Navigate to workspace
cd "$WORKSPACE_DIR" || { echo "Error: Workspace directory not found: $WORKSPACE_DIR"; exit 1; }

# 2. Source Environment
echo "[1/2] Sourcing ROS 2 dependencies..."
if [ -f "/home/nvidia/yanbo/gikWBC9DOF/scripts/orin/recomo_env.bash" ]; then
    source /home/nvidia/yanbo/gikWBC9DOF/scripts/orin/recomo_env.bash
else
    echo "Warning: recomo_env.bash not found, using fallbacks."
    source /opt/ros/humble/setup.bash
    [ -f "/home/nvidia/recomoArm_ws/install/setup.bash" ] && source /home/nvidia/recomoArm_ws/install/setup.bash
    [ -f "/home/nvidia/yanbo/ros2_ws/install/setup.bash" ] && source /home/nvidia/yanbo/ros2_ws/install/setup.bash
fi

# 3. Build
echo "[2/2] Building package: $PROJECT_NAME..."
colcon build --packages-select "$PROJECT_NAME" --symlink-install

echo "Build Success!"
