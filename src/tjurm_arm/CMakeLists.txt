cmake_minimum_required(VERSION 3.1.3)
project(tjurm_arm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  serial
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs   actionlib   actionlib_msgs serial
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_ros
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
)
include_directories(
  include
)

# add_executable(action_server2 src/action_server2.cpp)
# target_link_libraries(action_server2 ${catkin_LIBRARIES})

add_executable(joint_state_publisher src/joint_state_publisher.cpp)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES})
add_dependencies(joint_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

