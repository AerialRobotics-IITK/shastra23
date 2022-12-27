cmake_minimum_required(VERSION 2.8.3)
project(aruco_detector)

add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
add_message_files(
  FILES
  aruco_detected.msg
  aruco_message.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    cv_bridge
    image_transport
    sensor_msgs
    message_runtime
)



include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(aruco_detector_node src/aruco_detector_node.cpp src/aruco_detector.cpp)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
add_dependencies(aruco_detector_node aruco_detector_generate_messages_cpp)
target_link_libraries(aruco_detector_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
