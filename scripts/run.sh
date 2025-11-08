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
  --volume /dev/:/dev \
  --volume /run/udev:/run/udev \
  --rm \
  ${DOCKER_IMAGE_NAME} /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /ros2_shared_ws/install/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    ros2 launch waver_cv_bringup main_gscam.launch.py"

# Alternative launch commands:
# ros2 launch waver_cv_bringup main_gscam.launch.py      # Optical flow C++ in GStreamer (FAST - current)
# ros2 launch waver_cv_bringup mediapipe_face.launch.py  # MediaPipe face detection
# ros2 launch waver_cv_bringup mediapipe_hands.launch.py # MediaPipe hand tracking
# ros2 launch waver_cv_bringup optical_flow.launch.py    # Optical flow Python (slower)
# ros2 launch waver_cv_bringup sparse_optical_flow.launch.py  # Sparse optical flow Python
