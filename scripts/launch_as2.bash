#!/usr/bin/env bash
#
# launch_as2.bash — Launch AeroStack2 + Gazebo for the 3-drone swarm world
# used by both formation_control/ and dynamic_reallocation/.
#
# Usage:
#   ./launch_as2.bash -m      Multi-drone mode (drone0, drone1, drone2)
#
# NOTE: This is a thin repo-level wrapper. Adjust WORKSPACE and WORLD_CONFIG
# below to match your own AeroStack2 workspace layout, or replace this
# script's body with your AeroStack2 project's own as2_launch invocation.

set -e

WORKSPACE="${AEROSTACK2_WS:-$HOME/aerostack2_ws}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_CONFIG="${WORLD_CONFIG:-$PROJECT_ROOT/dynamic_reallocation/config/world_swarm.yaml}"

MULTI_DRONE=false

while getopts "m" opt; do
    case "$opt" in
        m) MULTI_DRONE=true ;;
        *) ;;
    esac
done

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "Launching AeroStack2 + Gazebo (multi-drone: $MULTI_DRONE)"
echo "World config: $WORLD_CONFIG"

# Replace the line below with your AeroStack2 launch invocation, e.g.:
# ros2 launch as2_gazebo_assets launch_simulation.py \
#     world:="$WORLD_CONFIG" \
#     drones:="drone0,drone1,drone2"

echo "TODO: point this script at your AeroStack2 launch file."
