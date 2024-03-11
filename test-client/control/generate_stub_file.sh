#!/bin/bash

# Get the directory of the script
SCRIPT_DIR=$(dirname "$0")

# Path to the motion_control.proto file relative to the script directory
PROTO_FILE_RELATIVE_PATH=../../PhysicsService/protos

# Resolve the absolute path of the proto file
PROTO_FILE_ABSOLUTE_PATH=$(readlink -f "$SCRIPT_DIR/$PROTO_FILE_RELATIVE_PATH")

# Determine the output directory for the generated Python stub file
OUTPUT_DIR="$SCRIPT_DIR"

# Generate the Python stub file from the proto file
protoc -I="$PROTO_FILE_ABSOLUTE_PATH" --python_out="$SCRIPT_DIR" motion_control.proto

echo "Python stub file generated successfully!"
