cmake_minimum_required(VERSION 3.2)
project(KaCanOpen_examples CXX)

find_package(Threads)

file(GLOB TARGETS_SRC
    "*.cpp"
)

if (${NO_ROS})
	list(REMOVE_ITEM TARGETS_SRC "${CMAKE_CURRENT_LIST_DIR}/ros_example.cpp")
    list(REMOVE_ITEM TARGETS_SRC "${CMAKE_CURRENT_LIST_DIR}/ros_joint_state_example.cpp")
endif()

foreach(TARGET_SRC ${TARGETS_SRC})

    get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
    add_executable(${TARGET_NAME} ${TARGET_SRC})
    set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)
    target_link_libraries(${TARGET_NAME} kacanopen_core)
    target_link_libraries(${TARGET_NAME} kacanopen_profiles)

	if (NOT(${NO_ROS}))
		target_link_libraries(${TARGET_NAME} kacanopen_ros)
	endif()
    
    
endforeach(TARGET_SRC ${TARGETS_SRC})

