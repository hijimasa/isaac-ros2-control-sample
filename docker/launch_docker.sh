#!/bin/bash
xhost +local:

HOST_GID=$(stat -c %g "$(pwd)/../colcon_ws")

docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host --privileged \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -u 1234:1234 \
    --group-add $HOST_GID \
    --env="DISPLAY" \
    -v $HOME/.Xauthority:/isaac-sim/.Xauthority:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --workdir="/isaac-sim/colcon_ws" \
    --volume="$(pwd)/../colcon_ws:/isaac-sim/colcon_ws" \
    isaac-ros2-image:latest	
