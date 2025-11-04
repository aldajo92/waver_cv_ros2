#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

echo "Checking ROS2 topics..."

docker exec -it ${DOCKER_CONTAINER_NAME} /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /ros2_shared_ws/install/setup.bash && \
    source /camera_ws/install/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    echo '=== Available topics ===' && \
    ros2 topic list && \
    echo '' && \
    echo '=== Topic info for /camera/image_raw ===' && \
    ros2 topic info /camera/image_raw && \
    echo '' && \
    echo '=== Topic hz for /camera/image_raw ===' && \
    timeout 5 ros2 topic hz /camera/image_raw || echo 'No messages received'"


