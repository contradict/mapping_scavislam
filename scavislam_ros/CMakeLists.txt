cmake_minimum_required(VERSION 2.8.3)
project(scavislam_ros)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

SET(CMAKE_VERBOSE_MAKEFILE ON)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        dynamic_reconfigure
        ScaViSLAM
        image_geometry
        image_transport
        cv_bridge
        pcl_ros
        eigen_conversions
        tf_conversions
    )

find_package(CUDA REQUIRED)
SET (OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 2.4.7 REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/stereo_vslam_node.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
    INCLUDE_DIRS
        include
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(scavislammessagehelper
    src/stereograph.cpp
    )

CUDA_ADD_EXECUTABLE(stereo_vslam_node
    src/slam_node.cpp
    )
target_link_libraries(stereo_vslam_node
    scavislammessagehelper
    ${catkin_LIBRARIES}
    )
