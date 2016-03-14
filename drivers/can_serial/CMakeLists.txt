cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring serial driver")

set(LIB_NAME can_serial)
set(HUB_NAME can_serial_hub)

add_library(${LIB_NAME} SHARED can_serial.c)

add_definitions("-fno-strict-aliasing")
add_executable(${HUB_NAME} can_serial_hub.c)
target_link_libraries(${HUB_NAME} ${LIB_NAME})

install(TARGETS ${LIB_NAME} ${HUB_NAME}
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
)

# Install
kacanopen_install("${LIB_NAME};${HUB_NAME}" FALSE)
