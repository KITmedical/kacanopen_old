cmake_minimum_required(VERSION 3.2)

message(STATUS "Configuring master")

file(GLOB TARGET_SRC
    "src/*.cpp"
)

# Create library
set(TARGET_NAME kacanopen_master)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)
target_link_libraries(${TARGET_NAME} kacanopen_core)

# Install
kacanopen_install(${TARGET_NAME} TRUE)