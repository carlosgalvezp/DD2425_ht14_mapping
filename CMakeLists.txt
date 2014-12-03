cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

project(robot_maps)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ras_utils)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs
	INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# OpenCV
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries(${OpenCV_LIBRARIES})

FILE(GLOB HEADERS include/${PROJECT_NAME}/*.h)
FILE(GLOB SOURCES src/*.cpp)

add_executable(map_handler_node src/apps/map_handler_node.cpp ${HEADERS} ${SOURCES})
add_dependencies(map_handler_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_handler_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

