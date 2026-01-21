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
_source_if_exists() {
    local path="$1"
    if [ -f "$path" ]; then
        # shellcheck disable=SC1090
        source "$path"
        return 0
    fi
    return 1
}

if _source_if_exists "/home/nvidia/yanbo/gikWBC9DOF/scripts/orin/recomo_env.bash" || \
   _source_if_exists "/home/nvidia/yanbo/gikWBC9DOF/orin/recomo_env.bash" || \
   _source_if_exists "/home/nvidia/yanbo/ros2_ws/scripts/orin/recomo_env.bash"; then
    true
else
    echo "Warning: recomo_env.bash not found, using fallbacks."
    source /opt/ros/humble/setup.bash
    [ -f "/home/nvidia/recomoArm_ws/install/setup.bash" ] && source /home/nvidia/recomoArm_ws/install/setup.bash
    [ -f "/home/nvidia/yanbo/ros2_ws/install/setup.bash" ] && source /home/nvidia/yanbo/ros2_ws/install/setup.bash
fi

# Optional RS4 overlay (ronin_rs4_driver).
RS4_WS_CANDIDATES=(
    "${RONIN_RS4_WS:-}"
    "${RS4_WS:-}"
    "/home/nvidia/jay/gimbal-v1-rs4/ros2_ws"
    "/home/nvidia/recomoArm_ws"
)

_find_rs4_ws() {
    local ws
    for ws in "${RS4_WS_CANDIDATES[@]}"; do
        if [ -n "$ws" ] && { [ -d "$ws/install/ronin_rs4_driver" ] || [ -d "$ws/install/share/ronin_rs4_driver" ]; }; then
            echo "$ws"
            return 0
        fi
    done
    for ws in "${RS4_WS_CANDIDATES[@]}"; do
        if [ -n "$ws" ] && [ -d "$ws/src/ronin_rs4_driver" ]; then
            echo "$ws"
            return 0
        fi
    done
    return 1
}

rs4_ws="$(_find_rs4_ws || true)"
if [ -n "$rs4_ws" ] && [ -f "$rs4_ws/install/setup.bash" ]; then
    echo "Sourcing RS4 overlay: $rs4_ws"
    # shellcheck disable=SC1090
    source "$rs4_ws/install/setup.bash"
elif [ -n "$rs4_ws" ] && [ -d "$rs4_ws/src/ronin_rs4_driver" ]; then
    echo "Building RS4 driver in: $rs4_ws"
    (cd "$rs4_ws" && colcon build --packages-select ronin_rs4_driver --symlink-install)
    if [ -f "$rs4_ws/install/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "$rs4_ws/install/setup.bash"
    fi
fi

# 3. Build
echo "[2/2] Building package: $PROJECT_NAME..."
colcon build --packages-select "$PROJECT_NAME" --symlink-install

echo "Build Success!"
