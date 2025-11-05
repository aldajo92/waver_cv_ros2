#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

echo "Opening interactive shell with all ROS2 workspaces sourced..."
echo "Available commands: sros2 (source all), bros2 (build all)"
echo ""

docker run -it \
  --privileged \
  --name=${DOCKER_CONTAINER_NAME}_shell \
  --network ${DOCKER_NETWORK} \
  --volume ${PROJECT_ROOT}/ros2_ws:/ros2_ws \
  --volume /dev/:/dev \
  --volume /run/udev:/run/udev \
  --rm \
  ${DOCKER_IMAGE_NAME} /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /ros2_shared_ws/install/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    exec /bin/bash"


