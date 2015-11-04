cmake_minimum_required(VERSION 3.2)
project(CanFestival_driver_socket C)

set(TARGET_NAME can_socket)

file(GLOB TARGET_SRC
    "*.c"
)

add_library(${TARGET_NAME} SHARED ${TARGET_SRC})

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
