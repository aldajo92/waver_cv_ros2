#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

echo "Testing GStreamer pipeline..."

docker run \
  --privileged \
  --name=${DOCKER_CONTAINER_NAME}_test \
  --volume /dev/:/dev \
  --volume /run/udev:/run/udev \
  --rm \
  ${DOCKER_IMAGE_NAME} /bin/bash -c "\
    echo '=== Testing libcamera devices ===' && \
    libcamera-hello --list-cameras && \
    echo '' && \
    echo '=== Testing GStreamer pipeline ===' && \
    timeout 5 gst-launch-1.0 libcamerasrc num-buffers=10 ! video/x-raw,width=640,height=480,format=BGR ! videoconvert ! fakesink || echo 'Pipeline test failed'"


