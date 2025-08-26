// #include <ros/ros.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Vector3.h>

// int main(int argc, char** argv)
// {
//     // ROS 노드 초기화
//     ros::init(argc, argv, "odom_publisher_node");
//     ros::NodeHandle nh;

//     // odom 토픽에 Odometry 메시지를 publish할 퍼블리셔 생성
//     ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

//     // 메시지 publish 주기를 설정
//     ros::Rate rate(10); // 10 Hz

//     while (ros::ok())
//     {
//         // 현재 시간을 가져옴
//         ros::Time current_time = ros::Time::now();

//         // Odometry 메시지 생성
//         nav_msgs::Odometry odom;
//         odom.header.stamp = current_time;
//         odom.header.frame_id = "odom";

//         // 위치 설정
//         odom.pose.pose.position.x = 1.0;
//         odom.pose.pose.position.y = 2.0;
//         odom.pose.pose.position.z = 0.0;
//         odom.pose.pose.orientation.x = 0.0;
//         odom.pose.pose.orientation.y = 0.0;
//         odom.pose.pose.orientation.z = 0.0;
//         odom.pose.pose.orientation.w = 1.0;

//         // 속도 설정
//         odom.child_frame_id = "base_link";
//         odom.twist.twist.linear.x = 0.1;
//         odom.twist.twist.linear.y = 0.0;
//         odom.twist.twist.linear.z = 0.0;
//         odom.twist.twist.angular.x = 0.0;
//         odom.twist.twist.angular.y = 0.0;
//         odom.twist.twist.angular.z = 0.0;

//         // 메시지 publish
//         odom_pub.publish(odom);

//         // 다음 루프를 위해 대기
//         rate.sleep();
//     }

//     return 0;
// }
