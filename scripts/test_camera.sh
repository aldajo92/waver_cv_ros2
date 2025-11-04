#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

echo "Testing camera and GStreamer pipeline..."

docker run \
  --privileged \
  --name=${DOCKER_CONTAINER_NAME}_test \
  --volume /dev/:/dev \
  --volume /run/udev:/run/udev \
  --rm \
  ${DOCKER_IMAGE_NAME} /bin/bash -c "
    echo '=== Checking for camera devices ===' && \
    ls -la /dev/video* 2>/dev/null || echo 'No /dev/video devices found' && \
    echo '' && \
    echo '=== Testing libcamera-hello ===' && \
    timeout 3 libcamera-hello --list-cameras 2>&1 || echo 'libcamera-hello failed or no cameras' && \
    echo '' && \
    echo '=== Testing GStreamer with libcamerasrc ===' && \
    GST_DEBUG=2 timeout 10 gst-launch-1.0 -v libcamerasrc num-buffers=30 ! video/x-raw,width=640,height=480 ! videoconvert ! video/x-raw,format=RGB ! fakesink 2>&1 || echo 'GStreamer pipeline failed'"


