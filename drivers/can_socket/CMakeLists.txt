cmake_minimum_required(VERSION 3.2)

message(STATUS "Configuring socket driver")


set(LIB_NAME can_socket)

file(GLOB TARGET_SRC
    "*.c"
)

add_library(${LIB_NAME} SHARED ${TARGET_SRC})

# Install
kacanopen_install(${LIB_NAME} FALSE)
