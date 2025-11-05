FROM ros:humble-ros-base-jammy
ENV ROS_DISTRO=humble

RUN apt update && apt install -y \
    python3-rosinstall \
    git \
    nano \
    tmux \
    wget \
    curl \
    iputils-ping \
    net-tools

RUN apt update && apt install -y \
    libeigen3-dev

RUN sudo apt update && apt install -y \
    python3-shapely \
    python3-yaml \
    python3-requests \
    python3-colcon-clean \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions

######### Camera configuration #####
RUN apt update && apt install -y --no-install-recommends \
    meson \
    ninja-build \
    pkg-config \
    libyaml-dev \
    python3-yaml \
    python3-ply \
    python3-jinja2 \
    libevent-dev \
    libdrm-dev \
    libcap-dev \
    python3-pip \
    python3-opencv \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-x \
    && apt-get clean \
    && apt-get autoremove \
    && rm -rf /var/cache/apt/archives/* \
    && rm -rf /var/lib/apt/lists/*

# Install web_video_server for viewing camera feed in browser
RUN apt update && apt install -y ros-humble-web-video-server

######### End camera configuration #########

# Clone and build ros2_shared (required by gscam2)
RUN mkdir -p /ros2_shared_ws/src
RUN git clone https://github.com/ptrmu/ros2_shared.git /ros2_shared_ws/src/ros2_shared -b master
RUN bash -c "cd /ros2_shared_ws && source /opt/ros/humble/setup.bash && colcon build"

# Build main workspace
COPY ./ros2_ws /ros2_ws
WORKDIR /ros2_ws

# Build OpenCV face detection GStreamer plugin
RUN apt update && apt install -y cmake libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN cd /ros2_ws/gst_mediapipe_plugin && \
    mkdir -p build && cd build && \
    cmake .. && \
    make && \
    make install

RUN bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src --skip-keys='libcamera ros2_shared' -y"
RUN bash -c "source /opt/ros/humble/setup.bash && source /ros2_shared_ws/install/setup.bash && colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias sros2='source /opt/ros/humble/setup.bash && source /ros2_shared_ws/install/setup.bash && source /ros2_ws/install/setup.bash'" >> ~/.bashrc
RUN echo "alias bros2='cd /ros2_ws && source /opt/ros/humble/setup.bash && source /ros2_shared_ws/install/setup.bash && colcon build'" >> ~/.bashrc

# COPY ./autostart.sh /autostart.sh
# RUN chmod +x /autostart.sh

# ENTRYPOINT [ "/autostart.sh" ]
