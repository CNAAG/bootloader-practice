#!/bin/bash
set -e

# Define directories
BUILD_DIR=./build
OUTPUT_DIR=./output
TOOLCHAIN_FILE=./cmake/toolchain-arm-none-eabi.cmake
CMAKE_OPTIONS="-DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE -DCMAKE_BUILD_TYPE=Release -S . -B ${BUILD_DIR}"
MAKE_OPTIONS="--build ${BUILD_DIR} --config Release -- -j$(nproc)"

# Create build and output directories if they don't exist
mkdir -p $BUILD_DIR
mkdir -p $OUTPUT_DIR
# rm -rf $BUILD_DIR/*
rm -rf $OUTPUT_DIR/*
echo "Cleaned $OUTPUT_DIR directories."
# echo "Cleaned $BUILD_DIR and $OUTPUT_DIR directories."

# Run CMake configuration
echo "Running CMake configuration..."
cmake $CMAKE_OPTIONS
echo "CMake configuration completed."

sleep 0.5

# Build the project
echo "Building the project..."
cmake $MAKE_OPTIONS
echo "Build completed."
