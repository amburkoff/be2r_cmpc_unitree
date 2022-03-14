include_directories("../third-party")
include_directories(SYSTEM "../third-party/qpOASES/include")
include_directories("../third-party/ParamHandler")
include_directories("fsm_states")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")  
include_directories("../controllers/convexMPC")
include_directories("../controllers")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources "fsm_states/*.cpp" "*.cpp")

add_library(fsm_lib ${sources})
target_link_libraries(fsm_lib WBC_Ctrl common_lib qpOASES controllers_lib)

# add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
# target_link_libraries(mit_ctrl robot biomimetics)
# target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
# target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
# target_link_libraries(mit_ctrl WBC_Ctrl)
# target_link_libraries(mit_ctrl VisionMPC)
# target_link_libraries(mit_ctrl ${catkin_LIBRARIES})
# add_dependencies(mit_ctrl ${PROJECT_NAME}_gencfg)
