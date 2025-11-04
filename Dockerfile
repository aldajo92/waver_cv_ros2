FROM ros:humble-ros-base-jammy
ENV ROS_DISTRO humble

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
    && apt-get clean \
    && apt-get autoremove \
    && rm -rf /var/cache/apt/archives/* \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/raspberrypi/libcamera.git /libcamera && cd /libcamera && git checkout 6ddd79b && cd /
RUN meson setup libcamera/build libcamera/
RUN ninja -C libcamera/build/ install

# Add the new installations to the python path so that picamera2 can find them
ENV PYTHONPATH=${PYTHONPATH}:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages:/app/kmsxx/build/py

# Finally install picamera2 using pip
RUN pip3 install picamera2

RUN apt update && apt install pkg-config python3-yaml python3-ply python3-jinja2 openssl libyaml-dev libssl-dev libudev-dev libatomic1 meson -y
RUN mkdir -p /camera_ws/src
RUN git clone https://github.com/christianrauch/camera_ros.git /camera_ws/src/camera_ros
RUN git clone https://github.com/RobotWebTools/web_video_server.git /camera_ws/src/web_video_server

RUN bash -c "cd /camera_ws/ && source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src --skip-keys=libcamera -y"
RUN bash -c "cd /camera_ws/ && source /opt/ros/humble/setup.bash && colcon build"


######### End camera configuration #########
COPY ./ros2_ws /ros2_ws
WORKDIR /ros2_ws
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias sros2='source /opt/ros/humble/setup.bash && source /camera_ws/install/setup.bash && source /ros2_ws/install/setup.bash'" >> ~/.bashrc
RUN echo "alias bros2='cd /ros2_ws && source /opt/ros/humble/setup.bash && source /camera_ws/install/setup.bash && colcon build'" >> ~/.bashrc

# COPY ./autostart.sh /autostart.sh
# RUN chmod +x /autostart.sh

# ENTRYPOINT [ "/autostart.sh" ]
