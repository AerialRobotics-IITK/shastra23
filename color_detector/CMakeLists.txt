cmake_minimum_required(VERSION 2.8.3)
project(color_detector)

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

add_executable(color_detector_node src/color_detector_node.cpp src/color_detector.cpp )
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(color_detector_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
