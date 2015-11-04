cmake_minimum_required(VERSION 3.2)
project(CanFestival_driver_serial C)

set(LIB_NAME can_serial)
set(HUB_NAME can_serial_hub)

add_library(${LIB_NAME} SHARED can_serial.c)

add_definitions(${CMAKE_CXX_FLAGS} "-fno-strict-aliasing")
add_executable(${HUB_NAME} can_serial_hub.c)
target_link_libraries(${HUB_NAME} ${LIB_NAME})

install(TARGETS ${LIB_NAME} ${HUB_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
