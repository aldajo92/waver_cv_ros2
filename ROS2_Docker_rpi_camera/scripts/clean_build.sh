#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"

sudo rm -rf ${PROJECT_ROOT}/ros2_ws/build 
sudo rm -rf ${PROJECT_ROOT}/ros2_ws/log 
sudo rm -rf ${PROJECT_ROOT}/ros2_ws/install
