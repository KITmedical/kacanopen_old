cmake_minimum_required(VERSION 3.2)
project(kacanopen C CXX)


### settings ###

include(CMakeSettings.txt)
set(CAN_DRIVER can_${CAN_DRIVER_NAME})
message(STATUS "Using CAN driver ${CAN_DRIVER}")

if(${BUILD_ALL_DRIVERS})
    message(STATUS "Building all available drivers.")
endif()

if(${CMAKE_BUILD_TYPE} MATCHES "Release")
    message(STATUS "You are in release mode.")
else()
    if(${EXHAUSTIVE_DEBUGGING})
        message(STATUS "Enabled exhaustive debugging.")
        add_definitions("-DEXHAUSTIVE_DEBUGGING")
    else()
        message(STATUS "Enabled debugging.")
    endif()
endif()


### GCC ist not supported until 4.9 ###

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
        message(FATAL_ERROR "GCC version must be at least 4.9. Earlier versions miss std::regex support.")
    endif()
endif()



### dependencies ###

find_package(Threads)

find_package(Boost 1.46.1 REQUIRED)     

# Boost required for property_tree / ini_parser
if(NOT(Boost_FOUND))
    message(FATAL_ERROR "Boost > 1.46.1 not found. Boost is required for ini_parser class. This is a header-only library. You can simply download the latest boost package from http://www.boost.org/, extract it into any directory and include it using the following catkin/CMake argument: -DBOOST_ROOT=\"/path/to/boost/\"")
endif()

message(STATUS "Found Boost headers in ${Boost_INCLUDE_DIRS}")
include_directories(${Boost_INCLUDE_DIRS})



### includes ###

include_directories(${CMAKE_CURRENT_LIST_DIR}/core/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/master/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/ros_bridge/include)

if (NOT(${NO_ROS}))

    catkin_package(
       INCLUDE_DIRS core/include master/include ros_bridge/include
       LIBRARIES ${CAN_DRIVER} kacanopen_core kacanopen_profiles kacanopen_ros
       CATKIN_DEPENDS message_runtime
    )

endif()


### Installation ###

# argument 1: targets
# argument 2: if true, install ${CMAKE_CURRENT_LIST_DIR}/include/
# call this from subproject's CMake files
macro(kacanopen_install)
    message(STATUS "Configuring installation for target(s) ${ARGV0}")
    if (${NO_ROS})
        install(TARGETS ${ARGV0}
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
        )
        if (${ARGV1})
            install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
                DESTINATION include/kacanopen
            )
        endif()
    else()
        install(TARGETS ${ARGV0}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION bin
        )
        if (${ARGV1})
            install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include
                DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            )
        endif()
    endif()
endmacro()


### Source code ###

if (NOT(${NO_ROS}))
    add_subdirectory(ros_bridge)
endif()

add_subdirectory(drivers)
add_subdirectory(core)
add_subdirectory(master)
add_subdirectory(examples)


