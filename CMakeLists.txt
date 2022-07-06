cmake_minimum_required(VERSION 3.3.2)

set (CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(alfa_gs) 


set(project_SOURCES main.cpp
    src/alfa_node.cpp src/alfa_gs.cpp
    lib/alfa_node.h lib/alfa_gs.h
    Labeler/label.cpp Labeler/label.h
    lib/utils.h
    )


set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_NO_SYSTEM_FROM_IMPORTED ON) # UI files
set(CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES ON) # UI files
set(CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES ON) # UI files
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(lib/ src/ Labeler/)

find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_filters
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
  alfa_msg
  cv_bridge
  image_transport
  

)
find_package(Boost REQUIRED COMPONENTS filesystem thread)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nodelet
    nodelet_topic_tools
    pcl_conversions
    pcl_msgs
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
  DEPENDS
    Boost
    PCL
    message_runtime
)

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
