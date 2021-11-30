/*
 * This node converts commands from brics_actuator::JointVelocity type to
 * individual joint's velocity command for simulated youbot arm.
 */

#include <ros/ros.h>
#include <brics_actuator/JointVelocities.h>
#include <std_msgs/Float64.h>

std::vector<ros::Publisher> joint_pubs;
ros::Subscriber joint_vel_sub;
ros::Timer step_timer;
size_t num_of_joints = 5;
std::vector<float> joint_vel;
std::vector<std::string> expected_joint_uri = {"arm_joint_1", "arm_joint_2", "arm_joint_3",
                                      "arm_joint_4", "arm_joint_5"};
std::string expected_unit = "s^-1 rad";

void jointVelCb(const brics_actuator::JointVelocities::ConstPtr& msg)
{
    if ( msg->velocities.size() != num_of_joints )
    {
        ROS_ERROR_STREAM("Number of joints do not match. Expecting "
                         << num_of_joints << ", got " << msg->velocities.size() << ".");
        return;
    }

    for ( size_t i = 0; i < num_of_joints; i++ )
    {
        if ( msg->velocities[i].joint_uri != expected_joint_uri[i] )
        {
            ROS_ERROR_STREAM("joint_uri does not match. Expecting "
                             << expected_joint_uri[i] << ", got "
                             << msg->velocities[i].joint_uri << ".");
            for ( size_t j = 0; j < num_of_joints; j++ )
            {
                joint_vel[j] = 0.0f;
            }
            return;
        }

        if ( msg->velocities[i].unit != expected_unit )
        {
            ROS_ERROR_STREAM("unit does not match. Expecting "
                             << expected_unit << ", got "
                             << msg->velocities[i].unit << ".");
            for ( size_t j = 0; j < num_of_joints; j++ )
            {
                joint_vel[j] = 0.0f;
            }
            return;
        }

        joint_vel[i] = msg->velocities[i].value;
    }
};

void step()
{
    for ( size_t i = 0; i < num_of_joints; i++ )
    {
        std_msgs::Float64 command_msg;
        command_msg.data = joint_vel[i];
        joint_pubs[i].publish(command_msg);
    }
};

void init()
{
    ros::NodeHandle nh("~");
    std::string arm_name;
    nh.param<std::string>("arm_name", arm_name, "arm_1");
    std::string arm_joint_vel_topic = "/" + arm_name + "/arm_controller/velocity_command";

    joint_vel = std::vector<float>(5, 0.0f);

    for ( size_t i = 0; i < num_of_joints; i++ )
    {
        std::stringstream joint_command_topic;
        joint_command_topic << "/" << arm_name << "/joint_" << i+1 << "_velocity/command";
        joint_pubs.push_back(nh.advertise<std_msgs::Float64>(joint_command_topic.str(), 1));
    }

    /* subscribers */
    joint_vel_sub = nh.subscribe(arm_joint_vel_topic, 5, jointVelCb);

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_joint_vel_convertor");
    init();
    ros::NodeHandle nh("~");

    /* ros params */
    float loop_rate;
    nh.param<float>("loop_rate", loop_rate, 10.0f);
    ros::Rate rate(loop_rate);

    while ( ros::ok() )
    {
        step();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
