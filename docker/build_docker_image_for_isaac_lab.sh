#!/bin/bash
#
# Build Docker image for Isaac Lab environment
#
# This script builds a Docker image that includes:
# - Isaac Sim 5.1.0
# - Isaac Lab (robot learning framework)
# - ROS 2 Jazzy
#
# Usage:
#   ./build_docker_image_for_isaac_lab.sh
#
# Note: Building may take a while due to Isaac Lab installation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=============================================="
echo " Building Isaac Lab Docker Image"
echo "=============================================="
echo ""
echo "This will build a Docker image with:"
echo "  - Isaac Sim 5.1.0"
echo "  - Isaac Lab"
echo "  - ROS 2 Jazzy"
echo ""
echo "This may take a while..."
echo ""

docker build \
    --build-arg NUM_THREADS=8 \
    --rm \
    -t isaac-lab-ros2-image:latest \
    -f "${SCRIPT_DIR}/Dockerfile_IsaacLab" \
    "${SCRIPT_DIR}"

echo ""
echo "=============================================="
echo " Build Complete!"
echo "=============================================="
echo ""
echo "To run the container:"
echo "  ./launch_docker_for_isaac_lab.sh"
echo ""
