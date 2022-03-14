cmake_minimum_required(VERSION 3.0.2)
project(be2r_cmpc_unitree)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  unitree_legged_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# вынести пути к исходникам отдельных частей mit контроллера в отдельные паременные и уже их кидать в include_directories. Для наглядности

include_directories(
  src
  src/be2r_cmpc_unitree
  # src/mit
  src/lcm-types
  src/fsm
  src/fsm/fsm_states
  src/controllers
  src/controllers/convexMPC
  src/common
  src/common/FootstepPlanner
  src/common/include
  src/common/include/Controllers
  src/common/include/ControlParameters
  src/common/include/Dynamics
  src/common/include/Math
  src/common/include/SimUtilities
  src/common/include/SparseCMPC
  # src/mit/common/include/Utilities
  src/third_party
  src/third_party/ParamHandler
  src/third_party/qpOASES/include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
)

set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
-Winit-self -Woverloaded-virtual \
-Wsign-promo")

set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")

SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_BINARY_DIR}/Configuration.h)

add_subdirectory(src/third_party)
add_subdirectory(src/common)
add_subdirectory(src/controllers)
add_subdirectory(src/fsm)

#основная либа ноды
add_library(be2r_cmpc_unitree_lib src/be2r_cmpc_unitree/be2r_cmpc_unitree.cpp)
target_link_libraries(be2r_cmpc_unitree_lib ${catkin_LIBRARIES} ctrl_fsm_lib fsm_lib)

#бинарник ноды
add_executable(unitree_ctrl src/be2r_cmpc_unitree_node.cpp)

#зависимости ноды
add_dependencies(unitree_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#линкуем либы к бинарнику
target_link_libraries(unitree_ctrl ${catkin_LIBRARIES} be2r_cmpc_unitree_lib)
