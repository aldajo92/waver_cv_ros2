#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

# docker run -it \
#   --privileged \
#   --name=${DOCKER_CONTAINER_NAME} \
#   --network ${DOCKER_NETWORK} \
#   --volume ${PROJECT_ROOT}/ros2_ws:/ros2_ws \
#   --volume /dev/:/dev \
#   --volume /run/udev:/run/udev \
#   --rm \
#   ${DOCKER_IMAGE_NAME} /bin/bash -c "\
#     source /ros2_ws/install/setup.bash && \
#     ros2 launch waver_cv_bringup main_gscam.launch.py"

docker run -it \
  --privileged \
  --name=${DOCKER_CONTAINER_NAME} \
  --network ${DOCKER_NETWORK} \
  --volume ${PROJECT_ROOT}/ros2_ws:/ros2_ws \
  --volume /dev/:/dev \
  --volume /run/udev:/run/udev \
  --rm \
  ${DOCKER_IMAGE_NAME} /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /ros2_shared_ws/install/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    ros2 launch waver_cv_bringup mediapipe_hands.launch.py"

# Alternative launch commands:
# ros2 launch waver_cv_bringup mediapipe_hands.launch.py # MediaPipe hand tracking (current)
# ros2 launch waver_cv_bringup mediapipe_face.launch.py  # MediaPipe face detection
# ros2 launch waver_cv_bringup main_gscam.launch.py      # gscam2 with OpenCV face (C++)
