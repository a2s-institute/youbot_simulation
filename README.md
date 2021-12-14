youbot_simulation
=================

Packages to run the KUKA youBot in the Gazebo simulation with ROS



#### Easy installation on ubuntu 20.04

```bash
cd ~/catkin_ws/src

# check https://github.com/mas-group/youbot_simulation/pull/19
# if it is accepted, then
git clone https://github.com/mas-group/youbot_simulation.git --branch noetic-devel
# else
git clone https://github.com/d3dx13/youbot_simulation.git


git clone https://github.com/mas-group/youbot_description.git

cd ~/catkin_ws
catkin build
source ~/.bashrc

```



#### Start simulation in one terminal

**OPTION 1:** Kuka manipulator only

```bash
killall rosmaster roscore gazebo roslaunch gzserver gzclient;
rosrun gazebo_ros gazebo &
roslaunch youbot_gazebo_robot youbot_arm_only.launch &

```

**OPTION 2:** Whole kuka youbot

```bash
killall rosmaster roscore gazebo roslaunch gzserver gzclient;
rosrun gazebo_ros gazebo &
roslaunch youbot_gazebo_robot youbot.launch &

```



#### Install some examples

```bash
cd ~/catkin_ws/src

# required library
git clone https://github.com/wnowak/brics_actuator.git

# examples
git clone https://github.com/d3dx13/youbot_ros_examples.git

cd ~/catkin_ws
catkin build
source ~/.bashrc

```



#### Launch some examples

Kuka manipulator moving

`rosrun youbot_ros_simple_trajectory youbot_ros_simple_trajectory`

YouBot platform moving (works only with **OPTION 2**)

`rosrun youbot_ros_hello_world youbot_ros_hello_world`



