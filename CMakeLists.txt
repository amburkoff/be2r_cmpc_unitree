cmake_minimum_required(VERSION 3.0.2)
project(be2r_cmpc_unitree)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES be2r_cmpc_unitree
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(be2r_cmpc_unitree_lib src/be2r_cmpc_unitree.cpp)
target_link_libraries(be2r_cmpc_unitree_lib ${catkin_LIBRARIES})

add_executable(be2r_unitree_ctrl src/be2r_cmpc_unitree_node.cpp)

add_dependencies(be2r_unitree_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(be2r_unitree_ctrl ${catkin_LIBRARIES} be2r_cmpc_unitree_lib)