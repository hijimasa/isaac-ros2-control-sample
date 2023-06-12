#!/bin/bash
#docker rmi irlab-image
docker build --build-arg NUM_THREADS=8 --rm -t isaac-ros2-image .
