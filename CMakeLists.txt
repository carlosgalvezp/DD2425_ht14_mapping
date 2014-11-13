cmake_minimum_required(VERSION 2.4.6)

project(robot_maps)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ras_utils)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs
	INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB_RECURSE SOURCES src/*.cpp)
FILE(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h)

add_executable(map_handler_node ${SOURCES} ${HEADERS})
add_dependencies(map_handler_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_handler_node ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
