include_directories(".")
include_directories("../../third-party/osqp/include")  # osqp includes
include_directories("FootstepPlanner")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources "Controllers/*.cpp" "SparseCMPC/*.cpp" "Dynamics/*.cpp" "Utilities/*.cpp" "ControlParameters/*.cpp" "debug/*.cpp" "SimUtilities/*.cpp" "reference_trajectory_generator/*.cpp")

add_library(common_lib ${sources})
target_link_libraries(common_lib ${catkin_LIBRARIES} osqp JCQP)

# Our libraries используется в vision mpc
# add_subdirectory(FootstepPlanner)

