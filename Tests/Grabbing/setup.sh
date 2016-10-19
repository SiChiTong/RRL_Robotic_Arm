#!/bin/bash

set -e
CURRENT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORLD_PATH="$CURRENT_PATH/plugins/world_control/build"
ARMWORLD_PATH="$CURRENT_PATH"
echo "Make in directory $WORLD_PATH" 
cd "$WORLD_PATH"
make
echo "Launching gazebo in directory $ARMWORLD_PATH"
cd "$ARMWORLD_PATH"
gazebo arm.world --verbose
