#!/usr/bin/env bash
#
# launch_ground_station.bash — Optional live ground-station viewer for
# monitoring drone state (alphanumeric viewer / mission executor) during
# a run. Used by both formation_control/ and dynamic_reallocation/.
#
# Usage:
#   ./launch_ground_station.bash -m -v
#     -m   multi-drone mode
#     -v   verbose

set -e

WORKSPACE="${AEROSTACK2_WS:-$HOME/aerostack2_ws}"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "Launching AeroStack2 ground station..."

# Replace with your AeroStack2 ground-station launch invocation, e.g.:
# ros2 launch as2_alphanumeric_viewer alphanumeric_viewer.launch.py "$@"

echo "TODO: point this script at your AeroStack2 ground-station launch file."
