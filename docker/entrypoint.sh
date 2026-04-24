#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash

# Change this to your SchurVINS repo_root on host.
# Make sure it's mounted to same path location in docker container
# and make sure this path is symlinked to /catkin_ws/src/SchurVins
# in docker container
SCHURVINS_HOST_PATH="<path/to/SchurVINS/repo_root>"

# Create results/logs dirs if missing
mkdir -p "$SCHURVINS_HOST_PATH/results" 2>/dev/null || true
mkdir -p "$SCHURVINS_HOST_PATH/logs" 2>/dev/null || true

# Source workspace if already built
if [ -f /catkin_ws/devel/setup.bash ]; then
    source /catkin_ws/devel/setup.bash
fi

exec "$@"
