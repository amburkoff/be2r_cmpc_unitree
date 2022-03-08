# include_directories("./")
include_directories("controllers")
include_directories("robot/include")
include_directories("common/include")
include_directories("common/FootstepPlanner")
include_directories("../third-party")
include_directories(SYSTEM "../third-party/qpOASES/include")
include_directories("../third-party/ParamHandler")
include_directories("fsm_states")

# include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources "fsm_states/*.cpp" 
                          "controllers/convexMPC/*.cpp"
                          "controllers/BackFlip/*.cpp")

# add_subdirectory(robot)
add_subdirectory(common)
add_subdirectory(controllers/WBC_Ctrl)
add_subdirectory(controllers/WBC)

add_library(ctrl_fsm_lib ${sources})
target_link_libraries(ctrl_fsm_lib WBC_Ctrl controllers_lib qpOASES)


# add_library(mit_ctrl_lib MIT_Controller.cpp ${sources})
# add_library(mit_ctrl_lib MIT_Controller.cpp)
# target_link_libraries(mit_ctrl_lib ${catkin_LIBRARIES} gait_lib ctrl_fsm_lib)
# target_link_libraries(mit_ctrl_lib ${catkin_LIBRARIES} controllers_lib ctrl_fsm_lib qpOASES)



# add_subdirectory(controllers/WBC)
# add_subdirectory(controllers/WBC_Ctrl)
# add_subdirectory(Controllers/VisionMPC)

# add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
# target_link_libraries(mit_ctrl robot biomimetics)
# target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
# target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
# target_link_libraries(mit_ctrl WBC_Ctrl)
# target_link_libraries(mit_ctrl VisionMPC)
# target_link_libraries(mit_ctrl ${catkin_LIBRARIES})
# add_dependencies(mit_ctrl ${PROJECT_NAME}_gencfg)
