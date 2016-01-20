cmake_minimum_required(VERSION 3.2)

message(STATUS "Configuring ros_bridge")

### catkin ###
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  # message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Threads)

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