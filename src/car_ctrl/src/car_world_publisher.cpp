#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/GetModelState.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_publisher");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  std::string model_name = "a_car";
  std::string relative_entity_name = "world";

  ros::Rate rate(10.0);
  while (nh.ok()) {
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model_name;
    srv.request.relative_entity_name = relative_entity_name;

    if (client.call(srv)) {
      transform.setOrigin(tf::Vector3(
        srv.response.pose.position.x,
        srv.response.pose.position.y,
        srv.response.pose.position.z
      ));
      q.setX(srv.response.pose.orientation.x);
      q.setY(srv.response.pose.orientation.y);
      q.setZ(srv.response.pose.orientation.z);
      q.setW(srv.response.pose.orientation.w);
      transform.setRotation(q);

      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
    } else {
      ROS_ERROR("Failed to call service /gazebo/get_model_state");
    }
    rate.sleep();
  }
  return 0;
}
