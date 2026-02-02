#!/bin/bash
#
# Launch Docker container for Isaac Lab environment
#
# This script starts a Docker container with:
# - Isaac Lab (official NVIDIA image)
# - ROS 2 Jazzy
# - GPU support
# - X11 display forwarding
#
# Usage:
#   ./launch_docker_for_isaac_lab.sh [--headless]
#
# Options:
#   --headless    Run in headless mode (no GUI)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEADLESS=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --headless)
            HEADLESS=true
            shift
            ;;
    esac
done

# Allow X server connections
xhost +local: 2>/dev/null || true

# Get host group ID for volume permissions
HOST_GID=$(stat -c %g "${SCRIPT_DIR}/../colcon_ws")

# Container name
CONTAINER_NAME="isaac-lab"

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '${CONTAINER_NAME}' already exists."
    echo "Attaching to existing container..."
    docker start -ai ${CONTAINER_NAME}
    exit 0
fi

echo "=============================================="
echo " Starting Isaac Lab Docker Container"
echo "=============================================="
echo ""
echo "Container: ${CONTAINER_NAME}"
echo "Headless:  ${HEADLESS}"
echo ""

# Build docker run command
# Note: Official Isaac Lab image runs as root
DOCKER_ARGS=(
    --name ${CONTAINER_NAME}
    --entrypoint bash
    -it
    --gpus all
    -e "ACCEPT_EULA=Y"
    --rm
    --network=host
    --privileged
    # Isaac Lab/Sim cache directories (official image paths)
    -v ~/docker/isaac-lab/cache/kit:/root/.cache/kit:rw
    -v ~/docker/isaac-lab/cache/ov:/root/.cache/ov:rw
    -v ~/docker/isaac-lab/cache/pip:/root/.cache/pip:rw
    -v ~/docker/isaac-lab/cache/nvidia:/root/.nv:rw
    -v ~/docker/isaac-lab/nvidia-omniverse:/root/.nvidia-omniverse:rw
    -v ~/docker/isaac-lab/local-share:/root/.local/share/ov:rw
    # Workspace volume (ROS 2 workspace) - mount to /root for root user
    --workdir="/root/colcon_ws"
    --volume="${SCRIPT_DIR}/../colcon_ws:/root/colcon_ws"
    # Isaac Lab checkpoints and logs
    -v ~/docker/isaac-lab/isaaclab_logs:/workspace/isaaclab/logs:rw
)

# Add display settings if not headless
if [ "$HEADLESS" = false ]; then
    DOCKER_ARGS+=(
        --env="DISPLAY"
        -v "$HOME/.Xauthority:/root/.Xauthority:rw"
        -v /tmp/.X11-unix:/tmp/.X11-unix
    )
else
    DOCKER_ARGS+=(
        -e "DISPLAY="
        -e "HEADLESS=1"
    )
fi

# Create necessary directories
mkdir -p ~/docker/isaac-lab/cache/kit
mkdir -p ~/docker/isaac-lab/cache/ov
mkdir -p ~/docker/isaac-lab/cache/pip
mkdir -p ~/docker/isaac-lab/cache/nvidia
mkdir -p ~/docker/isaac-lab/nvidia-omniverse
mkdir -p ~/docker/isaac-lab/local-share
mkdir -p ~/docker/isaac-lab/isaaclab_logs

# Run container
docker run "${DOCKER_ARGS[@]}" isaac-lab-ros2-image:latest

echo ""
echo "=============================================="
echo " Container Stopped"
echo "=============================================="
