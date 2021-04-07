cmake_minimum_required(VERSION 2.8.3)
project(zed-open-capture-ros)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        cv_bridge
        image_transport
        sensor_msgs
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /usr/local/include
    /usr/include/hidapi
    )

catkin_package(
    LIBRARIES 
    CATKIN_DEPENDS
)

add_executable(${PROJECT_NAME} src/zed_node.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    /usr/local/lib/libzed_open_capture.so
    )
