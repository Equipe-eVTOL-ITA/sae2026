#!/bin/bash
set -e

# ========== SAE 2026 — Build Script ==========

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [ $# -ne 1 ]; then
    echo "Usage: $0 <target>"
    echo "Targets:"
    echo "  all                 — build everything"
    echo "  deps                — build only dependencies (px4_msgs, fsm, custom_msgs, etc.)"
    echo "  stdstates           — build only stdstates"
    echo "  drone_lib           — build only drone_lib"
    echo "  audio_alert         — build only audio_alert"
    echo "  mission_1           — build only mission_1"
    echo "  mission_2           — build only mission_2"
    echo "  mission_3           — build only mission_3"
    exit 1
fi

source /opt/ros/humble/setup.bash

if [ -e "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

BUILD_TYPE=RelWithDebInfo

case $1 in
    all)
        colcon build \
            --symlink-install \
            --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
            -Wall -Wextra -Wpedantic \
            --executor sequential
        ;;
    deps)
        colcon build \
            --symlink-install \
            --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
            --packages-up-to stdstates \
            --executor sequential
        ;;
    stdstates)
        colcon build \
            --symlink-install \
            --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
            --packages-select stdstates \
            --executor sequential
        ;;
    drone_lib)
        colcon build \
            --symlink-install \
            --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
            --packages-select drone_lib \
            --executor sequential
        ;;
    mangueira_detector)
        colcon build \
            --symlink-install \
            --packages-select mangueira_detector \
            --executor sequential
        ;;
    mission_1)
        colcon build \
            --symlink-install \
            --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
            --packages-select mission_1 \
            --executor sequential
        ;;
    mission_2)
        colcon build \
            --symlink-install \
            --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
            --packages-select mission_2 \
            --executor sequential
        ;;
    mission_3)
        colcon build \
            --symlink-install \
            --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
            --packages-select mission_3 \
            --executor sequential
        ;;
    
    audio_alert)
        colcon build \
            --packages-select audio_alert \
            --symlink-install \
            --executor sequential
        ;;
    *)
        echo "Unknown target: $1"
        exit 1
        ;;
esac
