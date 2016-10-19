#!/bin/bash

set -e
CURRENT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORLD_PATH="$CURRENT_PATH/plugins/world_control/build"
echo "CMake in directory $WORLD_PATH" 
cd "$WORLD_PATH"
cmake ../
echo "You can run setup.sh"

