cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring ros_bridge")

file(GLOB TARGET_SRC
	"src/*.cpp"
)

# Create library
set(TARGET_NAME kacanopen_ros_bridge)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)
target_link_libraries(${TARGET_NAME} kacanopen_master)
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})

# Install
kacanopen_install(${TARGET_NAME} TRUE)
