cmake_minimum_required(VERSION 3.2)
project(KaCanOpen_profiles CXX)

find_package(Threads)

file(GLOB TARGET_SRC
    "src/*.cpp"
)

# Create library
set(TARGET_NAME kacanopen_profiles)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${TARGET_NAME} kacanopen_core)
