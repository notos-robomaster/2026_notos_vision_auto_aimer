#!/usr/bin/env bash

set -euo pipefail

PROJECT_DIR="/home/oconnor/Downloads/sp_vision_25-main"
LOG_DIR="$PROJECT_DIR/logs"

# Give desktop session and USB/camera services a few seconds to come up.
sleep 5

mkdir -p "$LOG_DIR"
cd "$PROJECT_DIR"

# Desktop autostart runs with a minimal environment; load ROS2 runtime paths.
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
fi

screen \
    -L \
    -Logfile "$LOG_DIR/$(date "+%Y-%m-%d_%H-%M-%S").screenlog" \
    -d \
    -m \
    bash -c "$PROJECT_DIR/build/auto_aim_test"
