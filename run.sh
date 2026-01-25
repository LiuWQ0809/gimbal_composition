#!/bin/bash
set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
PROJECT_NAME="cine_gimbal_control"

echo "=============================================="
echo "   Cine Gimbal Control - Run                  "
echo "=============================================="

# 1. Navigate to workspace
cd "$WORKSPACE_DIR" || { echo "Error: Workspace directory not found: $WORKSPACE_DIR"; exit 1; }

# 2. Source Environment
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
    source /opt/ros/humble/setup.bash
    [ -f "/home/nvidia/recomoArm_ws/install/setup.bash" ] && source /home/nvidia/recomoArm_ws/install/setup.bash
    [ -f "/home/nvidia/jay/gimbal-v1-rs4/ros2_ws/install/setup.bash" ] && source /home/nvidia/jay/gimbal-v1-rs4/ros2_ws/install/setup.bash
    # Verify/Add nanotrack_cpp workspace for recomo_msgs
    # [ -f "/home/nvidia/workspaces/perception/nanotrack_cpp/install/setup.bash" ] && source /home/nvidia/workspaces/perception/nanotrack_cpp/install/setup.bash
fi

# Verify/Add nanotrack_cpp workspace for recomo_msgs (Always source this)
[ -f "/home/nvidia/workspaces/perception/nanotrack_cpp/install/setup.bash" ] && source /home/nvidia/workspaces/perception/nanotrack_cpp/install/setup.bash

# Optional RS4 overlay (ronin_rs4_driver).
RS4_WS_CANDIDATES=(
    "${RONIN_RS4_WS:-}"
    "${RS4_WS:-}"
    "/home/nvidia/jay/gimbal-v1-rs4/ros2_ws"
    "/home/nvidia/recomoArm_ws"
    "/home/nvidia/yanbo/not-in-use/DJIgimbal/ros2_ws"
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
    echo "Warning: RS4 workspace found but not built: $rs4_ws"
fi

# 3. Source Local Workspace
if [ ! -f "install/setup.bash" ]; then
    echo "Error: install/setup.bash not found. Please run ./build.sh first."
    exit 1
fi
source install/setup.bash

# 4. Run (background)
LOG_FILE="${WORKSPACE_DIR}/cine_gimbal_control.log"
echo "Launching tracking node in background..."
nohup ros2 launch "$PROJECT_NAME" tracking.launch.py > "${LOG_FILE}" 2>&1 &
echo "PID: $!"
echo "Log: ${LOG_FILE}"
