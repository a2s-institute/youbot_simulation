youbot_simulation
=================

Packages to run the KUKA youBot in the Gazebo simulation with ROS2 Humble

To run the simulation for gazebo run the below commands

Terminal 1

```
source ~/ros_ws/install/setup.bash 
ros2 launch youbot_gazebo_robot start_world.launch.py
```

Terminal 2

```
source ~/ros_ws/install/setup.bash 
ros2 launch youbot_gazebo_robot spawn_youbot_ros2.launch.xml
```