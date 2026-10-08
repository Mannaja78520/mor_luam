#!/bin/bash
# Load ROS 2 Jazzy and the mor_luam workspace, then run the command.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
