include_directories(".")
include_directories("../third-party")
include_directories(SYSTEM "../third-party/qpOASES/include")
include_directories("../third-party/ParamHandler")
#include_directories("/usr/local/include/eigen3")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources "convexMPC/*.cpp"
                          "CMPC/*.cpp"
                          "BackFlip/*.cpp"
                          "BalanceController/*.cpp"
                          "VisionMPC/*.cpp")

add_subdirectory(WBC)
add_subdirectory(WBC_Ctrl)
# add_subdirectory(Controllers/VisionMPC)


add_library(controllers_lib ${sources})
target_link_libraries(controllers_lib WBC_Ctrl common_lib qpOASES)
