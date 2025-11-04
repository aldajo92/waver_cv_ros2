#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

docker run -it \
  --privileged \
  --name=${DOCKER_CONTAINER_NAME} \
  --network ${DOCKER_NETWORK} \
  --volume ${PROJECT_ROOT}/ros2_ws:/ros2_ws \
  --volume /dev/:/dev \
  --volume /run/udev:/run/udev \
  --rm \
  ${DOCKER_IMAGE_NAME} /bin/bash

echo "executing the command: docker run -it --privileged --name=${DOCKER_CONTAINER_NAME} --network ${DOCKER_NETWORK} --volume ${PROJECT_ROOT}/ros2_ws:/ros2_ws --volume /dev/:/dev --volume /run/udev:/run/udev --rm ${DOCKER_IMAGE_NAME} /bin/bash"

# ros2 launch camera_ros camera.launch.py width:=640 height:=480 format:=BGR888

# ros2 launch camera_ros camera.launch.py width:=1280 height:=720 format:=BGR888

  # -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
