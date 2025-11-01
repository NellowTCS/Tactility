#!/bin/sh

#
# Usage: release-simulator.sh [builddir] [target_path]
# Example: release-simulator.sh buildsim release/Simulator-linux-amd64
# Description: Releases the current simulator build files in the specified folder.
#

build_path=$1
target_path=$2

mkdir -p $target_path

cp version.txt $target_path

# Handle web (Emscripten) builds
if [ -f "$build_path/WebSimulator.js" ]; then
    echo "Releasing WebSimulator build..."
    cp $build_path/WebSimulator.js $target_path/
    cp $build_path/WebSimulator.wasm $target_path/
    cp $build_path/WebSimulator.html $target_path/
    # cp $build_path/WebSimulator.data $target_path/
else
    # Handle native builds
    echo "Releasing native simulator build..."
    cp $build_path/Firmware/FirmwareSim $target_path/
    cp -r Data/data $target_path/
    cp -r Data/system $target_path/
fi