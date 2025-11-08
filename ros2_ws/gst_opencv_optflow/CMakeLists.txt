cmake_minimum_required(VERSION 3.10)
project(gst_opencv_optflow)

set(CMAKE_CXX_STANDARD 14)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)
find_package(OpenCV REQUIRED)

include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
    ${GST_VIDEO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(gstopencvoptflow SHARED gst_opencv_optflow.cpp)

target_link_libraries(gstopencvoptflow
    ${GSTREAMER_LIBRARIES}
    ${GST_VIDEO_LIBRARIES}
    ${OpenCV_LIBS}
)

install(TARGETS gstopencvoptflow
    LIBRARY DESTINATION /usr/lib/aarch64-linux-gnu/gstreamer-1.0
)


