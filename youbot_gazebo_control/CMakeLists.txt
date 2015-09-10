cmake_minimum_required(VERSION 2.8.3)
project(youbot_gazebo_control)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    control_toolbox
    controller_interface
    geometry_msgs
    hardware_interface
    kdl_parser
    nav_msgs
    pluginlib
    realtime_tools
    roscpp
    roslint
    tf
    urdf
)

catkin_package(
  LIBRARIES
    steered_wheel_base_controller
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### LIBS
add_library(steered_wheel_base_controller
  src/steered_wheel_base_controller.cpp
)
target_link_libraries(steered_wheel_base_controller
  ${catkin_LIBRARIES}
)

### TESTS
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch)
endif()


### INSTALLS
install(TARGETS steered_wheel_base_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(FILES youbot_gazebo_control_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
