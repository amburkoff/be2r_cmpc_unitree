include_directories("../third-party")
include_directories(SYSTEM "../third-party/qpOASES/include")
include_directories("../third-party/ParamHandler")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")
include_directories(".")
# include_directories("WBC")
# include_directories("WBC/WBIC")
# include_directories("WBC_Ctrl")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources "convexMPC/*.cpp"
                          "BackFlip/*.cpp")

add_subdirectory(WBC)
add_subdirectory(WBC_Ctrl)
# add_subdirectory(Controllers/VisionMPC)


add_library(ctrl_fsm_lib ${sources})
target_link_libraries(ctrl_fsm_lib WBC_Ctrl controllers_lib qpOASES)

# add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
# target_link_libraries(mit_ctrl robot biomimetics)
# target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
# target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
# target_link_libraries(mit_ctrl WBC_Ctrl)
# target_link_libraries(mit_ctrl VisionMPC)
# target_link_libraries(mit_ctrl ${catkin_LIBRARIES})
# add_dependencies(mit_ctrl ${PROJECT_NAME}_gencfg)
