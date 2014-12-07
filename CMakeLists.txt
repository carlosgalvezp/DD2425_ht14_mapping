cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)

project(mapping)

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

# Eigen
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

FILE(GLOB HEADERS include/${PROJECT_NAME}/*.h)
FILE(GLOB SOURCES src/*.cpp)

add_executable(map_handler_node src/apps/map_handler_node.cpp ${HEADERS} ${SOURCES})
add_dependencies(map_handler_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_handler_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(map_loader_node src/apps/map_loader_node.cpp ${HEADERS} ${SOURCES})
add_dependencies(map_loader_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_loader_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(object_map_node src/apps/object_map_node.cpp ${HEADERS} ${SOURCES})
add_dependencies(object_map_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_map_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
