cmake_minimum_required(VERSION 3.2)

message(STATUS "Configuring virtual driver")


set(LIB_NAME can_virtual)

file(GLOB LIB_SRC
    "*.c"
)

add_library(${LIB_NAME} SHARED ${LIB_SRC})

# Install
kacanopen_install(${LIB_NAME} FALSE)
