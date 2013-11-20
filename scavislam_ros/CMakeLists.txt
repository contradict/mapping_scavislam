cmake_minimum_required(VERSION 2.8.3)
project(scavislam_ros)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

SET(CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED
    COMPONENTS
        dynamic_reconfigure
        ScaViSLAM
    )

find_package(CUDA REQUIRED)

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
