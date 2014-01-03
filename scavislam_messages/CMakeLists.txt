cmake_minimum_required(VERSION 2.8.3)
project(scavislam_messages)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        message_generation
        geometry_msgs
        std_msgs
        sensor_msgs
        )

add_message_files(
   FILES
   Point.msg
   Edge.msg
   Vertex.msg
   SLAMGraph.msg
)
generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(
)


