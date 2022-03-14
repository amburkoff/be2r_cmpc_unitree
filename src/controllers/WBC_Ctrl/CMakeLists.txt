FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)


add_library (WBC_Ctrl SHARED ${headers} ${sources} )
target_link_libraries (WBC_Ctrl controllers_lib lcm)
target_link_libraries (WBC_Ctrl WBIC dynacore_param_handler)

