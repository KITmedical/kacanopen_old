cmake_minimum_required(VERSION 3.2)

message(STATUS "Configuring peak_linux driver")

set(TARGET_NAME can_peak_linux)

# PCAN_PREFIX is an optional search path which can be defined by the user

find_path(PCAN_INCLUDE_DIR
    NAMES libpcan.h
    PATHS "${PCAN_PREFIX}/include"
)

find_library(PCAN_LIBRARY
    NAMES pcan
    PATHS "${PCAN_PREFIX}/lib"
)
    
if(NOT PCAN_INCLUDE_DIR OR NOT PCAN_LIBRARY)
    message("ERROR: PCAN library not found.")
    message("You should install PCAN or set PCAN_INCLUDE_DIR and PCAN_LIBRARY or PCAN_PREFIX manually.")
    message("See http://www.peak-system.com/fileadmin/media/linux/index.htm.")
    return()
else()
    message("-- PCAN include dir: ${PCAN_INCLUDE_DIR}")
    message("-- PCAN library: ${PCAN_LIBRARY}")
endif()

file(GLOB TARGET_SRC
    "*.c"
)

include_directories(${PCAN_INCLUDE_DIR})

add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
target_link_libraries(${TARGET_NAME} ${PCAN_LIBRARY})

# Install
kacanopen_install(${LIB_NAME} FALSE)
