cmake_minimum_required(VERSION 3.2)
project(CanFestival_driver_lincan C)

set(TARGET_NAME can_lincan)

file(GLOB TARGET_SRC
    "*.c"
)

add_library(${TARGET_NAME} SHARED ${TARGET_SRC})

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
