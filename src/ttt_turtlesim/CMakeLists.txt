cmake_minimum_required(VERSION 3.0.2)
project(ttt_turtlesim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  turtlesim
  message_generation
  actionlib_msgs
)

# Generate services
add_service_files(
  FILES
  ValidateMove.srv
  GetGameState.srv
)

# Generate actions
add_action_files(
  FILES
  ExecuteMove.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    turtlesim 
    message_runtime
    actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/game_manager.py
  scripts/move_executor.py
  scripts/game_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)