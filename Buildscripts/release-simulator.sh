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

# Check for WebSimulator (Emscripten) build
websim_js=$(find $build_path -name "WebSimulator.js" -print -quit)
if [ -n "$websim_js" ]; then
    echo "Releasing WebSimulator build..."
    js_dir=$(dirname "$websim_js")
    cp "$js_dir/WebSimulator.js" "$target_path/"
    cp "$js_dir/WebSimulator.wasm" "$target_path/"
    cp "$js_dir/WebSimulator.data" "$target_path/" 2>/dev/null || true # Optional data file
else
    # Handle native builds
    echo "Releasing native simulator build..."
    if [ -f "$build_path/Firmware/FirmwareSim" ]; then
        cp "$build_path/Firmware/FirmwareSim" "$target_path/"
        cp -r Data/data "$target_path/"
        cp -r Data/system "$target_path/"
    else
        echo "Error: FirmwareSim not found in $build_path. Please check the build output."
        exit 1
    fi
fi
