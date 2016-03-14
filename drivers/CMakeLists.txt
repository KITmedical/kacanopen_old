cmake_minimum_required(VERSION 2.8)
project(KaCanOpen_drivers C)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

if(WIN32)
    include_directories(${CMAKE_CURRENT_LIST_DIR}/include/win32)
else()
    include_directories(${CMAKE_CURRENT_LIST_DIR}/include/unix)
    include_directories(${CMAKE_CURRENT_LIST_DIR}/include/timers_unix)
endif()

if(${BUILD_ALL_DRIVERS})

    file(GLOB DRIVER_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/can_*"
    )

    foreach(DRIVER_DIR ${DRIVER_DIRS})
        add_subdirectory(${DRIVER_DIR})
    endforeach(DRIVER_DIR)
    
else()

    add_subdirectory(${CAN_DRIVER})
    
endif()
