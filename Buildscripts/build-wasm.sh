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

# Try to find emsdk_env.sh in common locations
EMSDK_ENV_SH=""

# Check environment variable first
if [ -n "$EMSDK" ] && [ -f "$EMSDK/emsdk_env.sh" ]; then
    EMSDK_ENV_SH="$EMSDK/emsdk_env.sh"
elif [ -f "/workspaces/emsdk/emsdk_env.sh" ]; then
    EMSDK_ENV_SH="/workspaces/emsdk/emsdk_env.sh"
elif [ -f "$HOME/emsdk/emsdk_env.sh" ]; then
    EMSDK_ENV_SH="$HOME/emsdk/emsdk_env.sh"
elif [ -f "/opt/emsdk/emsdk_env.sh" ]; then
    EMSDK_ENV_SH="/opt/emsdk/emsdk_env.sh"
else
    EMSDK_ENV_SH="$(command -v emsdk_env.sh 2>/dev/null || true)"
fi

if [ -z "$EMSDK_ENV_SH" ] || [ ! -f "$EMSDK_ENV_SH" ]; then
    echo "Error: Could not find emsdk_env.sh. Please ensure Emscripten SDK is installed and emsdk_env.sh is available in your PATH or a standard location."
    exit 1
fi

echo "Using Emscripten environment: $EMSDK_ENV_SH"
# shellcheck source=/dev/null
source "$EMSDK_ENV_SH"

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

    echo "Building FirmwareSim..."
    cmake --build build-sim-wasm --target FirmwareSim -j$(nproc)
else
    # If not cleaning, ensure CMake is configured if build dir was just created
    if [ ! -f build-sim-wasm/Makefile ] && [ ! -f build-sim-wasm/build.ninja ]; then
        echo "Configuring with CMake (initial)..."
        emcmake cmake -B build-sim-wasm -DCMAKE_BUILD_TYPE=Debug
    fi

    echo "Building FirmwareSim..."
    cmake --build build-sim-wasm --target FirmwareSim -j$(nproc)
fi

echo "Copying built files to Devices/websimulator/..."
cp build-sim-wasm/Firmware/FirmwareSim.* Devices/websimulator/Frontend

echo "Build complete!"