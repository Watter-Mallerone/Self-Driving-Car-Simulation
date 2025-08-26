#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/LinkStates.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>

std_msgs::Float64MultiArray imu_data;
void LinkState_CB(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    std::string link_name = "a_car::base_link";
    imu_data.data.resize(4);

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == link_name)
        {
            geometry_msgs::Pose link_pose = msg->pose[i];
            geometry_msgs::Twist link_vel = msg->twist[i];
            imu_data.data[0] = link_pose.orientation.x;
            imu_data.data[1] = link_pose.orientation.y;
            imu_data.data[2] = link_pose.orientation.z;
            imu_data.data[3] = link_pose.orientation.w;
            // imu_data.data[4] = link_vel.linear.x;
            // imu_data.data[5] = link_vel.linear.y;
            // imu_data.data[6] = link_vel.linear.z;
            // ROS_INFO("[Body Pose] x: %f, y: %f, x: %f", link_pose.position.x, link_pose.position.y, link_pose.position.z);
            break;
        }
    }
}

std_msgs::Int16 encoder_data;
void JointState_CB(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::string target_joint_name = "FR_wheel_joint";

    for(size_t i = 0; i < msg->name.size(); ++i)
    {
        if(msg->name[i] == target_joint_name)
        {
            double encoder_value = msg->position[i];
            int16_t encoder_value_int = static_cast<int16_t>(encoder_value);
            encoder_data.data = encoder_value_int;
            // ROS_INFO("Encoder value for : %d", encoder_data.data);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;

    ros::Subscriber LinkState_Sub = nh.subscribe("/gazebo/link_states", 1, LinkState_CB);
    ros::Subscriber JointState_sub = nh.subscribe("/a_car/joint_states", 1, JointState_CB);

    ros::Publisher IMU_Pub = nh.advertise<std_msgs::Float64MultiArray>("/imu_data", 1);
    ros::Publisher Encoder_Pub = nh.advertise<std_msgs::Int16>("/encoder_data", 1);

    ros::Rate rate(30.0);

    while (ros::ok()) {
    IMU_Pub.publish(imu_data);
    Encoder_Pub.publish(encoder_data);
    ros::spinOnce();
    rate.sleep();
    }

    return 0;
};