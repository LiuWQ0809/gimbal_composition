#!/bin/bash

# Configuration
NODE_NAME="tracking_node"

echo "=============================================="
echo "   Cine Gimbal Control - Stop                 "
echo "=============================================="

# Check if node is running
if pgrep -f "$NODE_NAME" > /dev/null; then
    echo "Found running process for $NODE_NAME. Stopping..."
    pkill -f "$NODE_NAME"
    echo "Signal sent."
else
    echo "No running process found for $NODE_NAME."
fi
