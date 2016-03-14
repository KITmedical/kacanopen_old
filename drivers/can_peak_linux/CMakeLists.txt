cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring peak_linux driver")

set(TARGET_NAME can_peak_linux)

# PCAN_PREFIX is an optional search path which can be defined by the user
message(STATUS "PCAN_PREFIX = ${PCAN_PREFIX}")

find_path(LIBPCAN_INCLUDE_DIR
    NAMES libpcan.h
    PATHS ${PCAN_PREFIX} ${PCAN_PREFIX}/include ${PCAN_PREFIX}/lib ${PCAN_PREFIX}/driver
)

find_path(PCAN_INCLUDE_DIR
    NAMES pcan.h
    PATHS ${PCAN_PREFIX} ${PCAN_PREFIX}/include ${PCAN_PREFIX}/lib ${PCAN_PREFIX}/driver
)

find_library(PCAN_LIBRARY
    NAMES pcan
    PATHS ${PCAN_PREFIX} ${PCAN_PREFIX}/lib
)
    
if((NOT PCAN_INCLUDE_DIR) OR (NOT PCAN_LIBRARY))
    message("ERROR: PCAN library not found.")
    message("You should install PCAN or set PCAN_INCLUDE_DIR and PCAN_LIBRARY or PCAN_PREFIX manually.")
    message("See http://www.peak-system.com/fileadmin/media/linux/index.htm.")
    return()
else()
    message("-- LIBPCAN include dir: ${LIBPCAN_INCLUDE_DIR}")
    message("-- PCAN include dir: ${PCAN_INCLUDE_DIR}")
    message("-- PCAN library: ${PCAN_LIBRARY}")
endif()

file(GLOB TARGET_SRC
    "*.c"
)

include_directories(${LIBPCAN_INCLUDE_DIR} ${PCAN_INCLUDE_DIR})

add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
target_link_libraries(${TARGET_NAME} ${PCAN_LIBRARY})

# Install
kacanopen_install(${TARGET_NAME} FALSE)
