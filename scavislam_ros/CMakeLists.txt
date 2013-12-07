cmake_minimum_required(VERSION 2.8.3)
project(scavislam_ros)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

SET(CMAKE_VERBOSE_MAKEFILE ON)

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
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

CUDA_ADD_EXECUTABLE(stereo_vslam_node
    src/slam_node.cpp
    )
target_link_libraries(stereo_vslam_node ${catkin_LIBRARIES})
