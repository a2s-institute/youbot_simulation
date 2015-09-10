cmake_minimum_required(VERSION 2.8.3)
project(youbot_gazebo_robot)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    robot_state_publisher
    youbot_gazebo_control
    youbot_gazebo_worlds
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch)
endif()

### INSTALLS
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
