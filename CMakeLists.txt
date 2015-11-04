cmake_minimum_required(VERSION 3.2)
project(KaCanOpen C CXX)


### settings ###

include(CMakeSettings.txt)
set(CAN_DRIVER can_${CAN_DRIVER_NAME})
message(STATUS "Using CAN driver ${CAN_DRIVER}")


### includes ###

include_directories(${CMAKE_CURRENT_LIST_DIR}/kacanopen_core/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/kacanopen_profiles/include)


### source ###

add_subdirectory(canfestival_drivers)
add_subdirectory(kacanopen_core)
add_subdirectory(kacanopen_profiles)
add_subdirectory(kacanopen_ros)
add_subdirectory(kacanopen_examples)
