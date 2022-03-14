include_directories(".")
include_directories("include/" )                       # common includes
include_directories("/usr/local/include/lcm/")         # lcm includes
include_directories("../../third-party/inih")          # inih includes
include_directories("../../third-party/osqp/include")  # osqp includes
include_directories("../../third-party/ParamHandler")
include_directories("/usr/local/include/eigen3")       # lcm includes
include_directories("FootstepPlanner")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources "src/Controllers/*.cpp" "src/SparseCMPC/*.cpp" "src/Dynamics/*.cpp" "src/Utilities/*.cpp" "src/ControlParameters/*.cpp")

add_library(controllers_lib ${sources})
target_link_libraries(controllers_lib ${catkin_LIBRARIES} osqp JCQP lcm inih dynacore_param_handler)

# Our libraries используется в vision mpc
# add_subdirectory(FootstepPlanner)

