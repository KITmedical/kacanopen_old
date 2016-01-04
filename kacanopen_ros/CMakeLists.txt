cmake_minimum_required(VERSION 3.2)
project(KaCanOpen_ros CXX)

# Dummy

#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/..)

### catkin ###

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
#    entry_data.msg
# Message1.msg
# Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    action.srv
    mode.srv
#   Service1.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kacanopen
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Threads)

file(GLOB TARGET_SRC
    "src/*.cpp"
)

# Create library
set(TARGET_NAME kacanopen_ros)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)
target_link_libraries(${TARGET_NAME} kacanopen_profiles)

target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})
#add_dependencies(${TARGET_NAME} ros_can_bridge_generate_messages_cpp)
