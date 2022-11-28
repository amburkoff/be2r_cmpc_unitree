include_directories(".")
include_directories("../third-party")
include_directories(SYSTEM "../third-party/qpOASES/include")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources "convexMPC/*.cpp"
                          # "CMPC/*.cpp"
                          "BackFlip/*.cpp"
                          "BalanceController/*.cpp"
                          "VisionMPC/*.cpp")

add_subdirectory(WBC)
add_subdirectory(WBC_Ctrl)
# add_subdirectory(Controllers/VisionMPC)

file(GLOB_RECURSE cmpc_lib_sources "CMPC/*.cpp")

add_library(cmpc_lib ${cmpc_lib_sources})
target_link_libraries(cmpc_lib common_lib qpOASES ${catkin_LIBRARIES})

add_library(controllers_lib ${sources})
target_link_libraries(controllers_lib WBC_Ctrl cmpc_lib ${catkin_LIBRARIES})

