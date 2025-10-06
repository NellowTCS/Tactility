#!/bin/bash

#
# Usage: build-wasm.sh [--clean]
# Example: ./build-wasm.sh --clean
# Description:
#   This script sets up the Emscripten environment and builds the AppSim WebAssembly (WASM) target.
#   If the '--clean' flag is provided, it removes the previous build directory and reconfigures with CMake.
#   After building, it copies the generated WASM, JS, data, and worker files to the WASM/ directory.
#   Ensure that the Emscripten SDK is installed at /workspaces/emsdk/emsdk_env.sh before running.
#
#   To serve the built files with pthread support (unused however), run:
#     cd WASM && python3 serve.py
#

set -e

CLEAN=0

if [[ "$1" == "--clean" ]]; then
    CLEAN=1
fi

# Get the script's directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Setting up Emscripten environment..."

if [ ! -f /workspaces/emsdk/emsdk_env.sh ]; then
    echo "Error: Emscripten SDK not found at /workspaces/emsdk/emsdk_env.sh"
    echo "Please ensure it is installed and the path is correct."
    exit 1
fi
source /workspaces/emsdk/emsdk_env.sh

cd "$PROJECT_ROOT"
# Ensure build directory exists
if [ ! -d build-sim-wasm ]; then
    mkdir -p build-sim-wasm
fi

if [[ $CLEAN -eq 1 ]]; then
    echo "Cleaning old build..."
    rm -rf build-sim-wasm
    mkdir -p build-sim-wasm

    echo "Configuring with CMake..."
    emcmake cmake -B build-sim-wasm -DCMAKE_BUILD_TYPE=Debug

    echo "Building AppSim..."
    cmake --build build-sim-wasm --target AppSim -j$(nproc)
else
    # If not cleaning, ensure CMake is configured if build dir was just created
    if [ ! -f build-sim-wasm/Makefile ] && [ ! -f build-sim-wasm/build.ninja ]; then
        echo "Configuring with CMake (initial)..."
        emcmake cmake -B build-sim-wasm -DCMAKE_BUILD_TYPE=Debug
    fi

    echo "Building AppSim..."
    cmake --build build-sim-wasm --target AppSim -j$(nproc)
fi

echo "Copying built files to Boards/WebSimulator/..."
cp build-sim-wasm/Firmware/AppSim.* Boards/WebSimulator/

echo "Build complete!"