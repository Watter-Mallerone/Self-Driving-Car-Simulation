#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

std_msgs::Float32MultiArray gps_cov;
std_msgs::Float64MultiArray gps_crds;

void LinkState_CB(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    std::string link_name = "a_car::base_link";
    gps_cov.data.resize(4);
    gps_crds.data.resize(2);

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == link_name)
        {
            geometry_msgs::Pose link_pose = msg->pose[i];
            gps_cov.data[0] = 0.5;
            gps_cov.data[1] = 0.5;
            gps_cov.data[2] = 0.5;
            gps_cov.data[3] = 0.5;
            gps_crds.data[0] = link_pose.position.x;
            gps_crds.data[1] = link_pose.position.y;
            // ROS_INFO("[Body Pose] x: %f, y: %f, x: %f", link_pose.position.x, link_pose.position.y, link_pose.position.z);
            break;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_publisher");
    ros::NodeHandle nh;

    ros::Subscriber LinkState_Sub = nh.subscribe("/gazebo/link_states", 1, LinkState_CB);

    ros::Publisher GPS_Cov_Pub = nh.advertise<std_msgs::Float32MultiArray>("/gps_cov", 1);
    ros::Publisher GPS_Pub = nh.advertise<std_msgs::Float64MultiArray>("/gps_crds", 1);

    ros::Rate rate(1);

    while (ros::ok()) {
    GPS_Cov_Pub.publish(gps_cov);
    GPS_Pub.publish(gps_crds);
    ros::spinOnce();
    rate.sleep();
    }

    return 0;
};