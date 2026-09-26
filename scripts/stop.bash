#!/usr/bin/env bash
#
# stop.bash — Stop the Gazebo simulation and associated AeroStack2 processes
# started by launch_as2.bash / launch_ground_station.bash.

echo "Stopping AeroStack2 and Gazebo processes..."

pkill -f "gzserver"        2>/dev/null || true
pkill -f "gzclient"        2>/dev/null || true
pkill -f "as2_"            2>/dev/null || true
pkill -f "ros2 launch"     2>/dev/null || true

echo "Done."
