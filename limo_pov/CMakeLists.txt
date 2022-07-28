cmake_minimum_required(VERSION 3.0.2)
project(limo_pov)

find_package(catkin REQUIRED COMPONENTS roscpp image_transport cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(lane_tracer scripts/lane_tracer.cpp)
target_link_libraries(lane_tracer ${catkin_LIBRARIES} ${OpenCV_LIBS})