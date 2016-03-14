cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring master")

file(GLOB TARGET_SRC
	"src/*.cpp"
)

# Create library
set(TARGET_NAME kacanopen_master)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)

# Boost filesystem required! See ../CMakeLists.txt
target_link_libraries(${TARGET_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} kacanopen_core)

# Install
kacanopen_install(${TARGET_NAME} TRUE)
