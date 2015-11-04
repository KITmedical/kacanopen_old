cmake_minimum_required(VERSION 3.2)
project(KaCanOpen_examples CXX)

find_package(Threads)

file(GLOB TARGETS_SRC
    "*.cpp"
)

foreach(TARGET_SRC ${TARGETS_SRC})

    get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
    add_executable(${TARGET_NAME} ${TARGET_SRC})
    set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)
    target_link_libraries(${TARGET_NAME} kacanopen_core)
    target_link_libraries(${TARGET_NAME} kacanopen_profiles)
    
endforeach(TARGET_SRC ${TARGETS_SRC})

