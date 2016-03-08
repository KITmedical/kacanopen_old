cmake_minimum_required(VERSION 3.2)

message(STATUS "Configuring lincan driver")

set(TARGET_NAME can_lincan)

file(GLOB TARGET_SRC
    "*.c"
)

add_library(${TARGET_NAME} SHARED ${TARGET_SRC})

# Install
kacanopen_install(${TARGET_NAME} FALSE)
