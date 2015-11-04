cmake_minimum_required(VERSION 3.2)
project(CanFestival_driver_virtual C)

set(LIB_NAME can_virtual)

file(GLOB LIB_SRC
    "*.c"
)

add_library(${LIB_NAME} SHARED ${LIB_SRC})

install(TARGETS ${LIB_NAME}
  LIBRARY DESTINATION lib
)
