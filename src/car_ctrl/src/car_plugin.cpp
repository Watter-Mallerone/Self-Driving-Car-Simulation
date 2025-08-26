#include <cmath>
#include <ros/ros.h>
#include <ros/master.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <controller_manager/controller_manager.h>
#include <position_controllers/joint_position_controller.h>

class MyRobotHW : public hardware_interface::RobotHW {
public:
    MyRobotHW() {
        // ####### Hardware Interface Setup ####### //
        hardware_interface::JointStateHandle FR_steer_handle("FR_steer_joint", &pos[0], &vel[0], &eff[0]);
        joint_state_interface.registerHandle(FR_steer_handle);
        hardware_interface::JointHandle FR_pos_handle(joint_state_interface.getHandle("FR_steer_joint"), &cmd[0]);
        position_joint_interface.registerHandle(FR_pos_handle); // Front Right Steer
        hardware_interface::JointStateHandle FL_steer_handle("FL_steer_joint", &pos[1], &vel[1], &eff[1]);
        joint_state_interface.registerHandle(FL_steer_handle);
        hardware_interface::JointHandle FL_pos_handle(joint_state_interface.getHandle("FL_steer_joint"), &cmd[1]);
        position_joint_interface.registerHandle(FL_pos_handle); // Front Left Steer

        hardware_interface::JointStateHandle FR_wheel_handle("FR_wheel_joint", &pos[2], &vel[2], &eff[2]);
        joint_state_interface.registerHandle(FR_wheel_handle);
        hardware_interface::JointHandle FR_vel_handle(joint_state_interface.getHandle("FR_wheel_joint"), &cmd[2]);
        velocity_joint_interface.registerHandle(FR_vel_handle);// Front Right Wheel
        hardware_interface::JointStateHandle FL_wheel_handle("FL_wheel_joint", &pos[3], &vel[3], &eff[3]);
        joint_state_interface.registerHandle(FL_wheel_handle);
        hardware_interface::JointHandle FL_vel_handle(joint_state_interface.getHandle("FL_wheel_joint"), &cmd[3]);
        velocity_joint_interface.registerHandle(FL_vel_handle); // Front Left Wheel
        hardware_interface::JointStateHandle RR_wheel_handle("RR_wheel_joint", &pos[4], &vel[4], &eff[4]);
        joint_state_interface.registerHandle(RR_wheel_handle);
        hardware_interface::JointHandle RR_vel_handle(joint_state_interface.getHandle("RR_wheel_joint"), &cmd[4]);
        velocity_joint_interface.registerHandle(RR_vel_handle);// Rear Right Wheel
        hardware_interface::JointStateHandle RL_wheel_handle("RL_wheel_joint", &pos[5], &vel[5], &eff[5]);
        joint_state_interface.registerHandle(RL_wheel_handle);
        hardware_interface::JointHandle RL_vel_handle(joint_state_interface.getHandle("RL_wheel_joint"), &cmd[5]);
        velocity_joint_interface.registerHandle(RL_vel_handle); // Raer Left Wheel

        registerInterface(&joint_state_interface);
        registerInterface(&position_joint_interface);
        registerInterface(&velocity_joint_interface);
    }

    void read()
    {

    }

    void write()
    {

    }

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;
    double cmd[6];
    double pos[6];
    double vel[6];
    double eff[6];
};

float Thrf = 0.0;
float Strf = 0.0;

float linear = 0.0;
float angle = 0.0;

float len = 0.75;
float wid = 0.55;
float pi = M_PI;
float limit = pi/3;

bool err_check1= true;
bool err_check2 = true;

void Ctrl_CB(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    Thrf = msg->data[0];
    Strf = msg->data[1];
    Thrf =  std::clamp(Thrf, -50.0f, 50.0f);
    // Strf =  std::clamp(Strf, 0, 0);
    // ROS_INFO("Throttle: %f Steering: %f",Thr, Str);
}

void ModelState_CB(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    geometry_msgs::Pose Body_Pos = msg->pose[1];
    geometry_msgs::Twist Body_Vel = msg->twist[1];
    tf2::Quaternion BQ(Body_Pos.orientation.x, Body_Pos.orientation.y, Body_Pos.orientation.z, Body_Pos.orientation.w);
    tf2::Matrix3x3 BM(BQ);
    double roll, pitch, yaw;
    BM.getRPY(roll, pitch, yaw);
    // ROS_INFO("[Body Euler Angle] roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    // ROS_INFO("[Body Linear Velocity] Vel_X: %f, Vel_Y: %f, Vel_Z: %f", Body_Vel.linear.x, Body_Vel.linear.y, Body_Vel.linear.z);
    // ROS_INFO("[Body Pose] x: %f, y: %f, x: %f", Body_Pos.position.x, Body_Pos.position.y, Body_Pos.position.z);
}

void LinkState_CB(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    std::string link_name = "a_car::FR_steer_link";

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == link_name)
        {
            geometry_msgs::Pose pose = msg->pose[i];
            tf2::Quaternion LQ(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            tf2::Matrix3x3 LM(LQ);
            double roll, pitch, yaw;
            LM.getRPY(roll, pitch, yaw);
            // ROS_INFO("[Link Euler Angle]roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
            break;
        }
    }
}

void Linear_CB(const std_msgs::Int16::ConstPtr& msg) {
    linear = msg->data;
    // ROS_INFO("linear: %f", linear);
}

void Angle_CB(const std_msgs::Int16::ConstPtr& msg) {
    angle = -(msg->data);
    // ROS_INFO("angle: %f", angle);
}

std::tuple<float, float, float> ackermann_steer(float ang) {
    ang = std::clamp(ang * pi/180, -limit, limit);
    // float R_str = atan((len*tan(angle))/(wid-0.5*0.55*wid*tan(angle)));
    // float L_str = atan((len*tan(angle))/(wid+0.5*0.55*wid*tan(angle)));
    float R_str =  std::clamp(atan((2*len*sin(ang))/(2*len*cos(ang)-wid*sin(ang))), -limit, limit);
    float L_str =  std::clamp(atan((2*len*sin(ang))/(2*len*cos(ang)+wid*sin(ang))), -limit, limit);
    // ROS_INFO("Angle : %f", ang * 180/pi);
    return std::make_tuple(R_str, L_str, ang);
}

bool state = false;
bool Connection_Check(ros::Subscriber& sub) {
    if (sub.getNumPublishers() > 0){
        state = true;
    }
    else {
        state = false;
    }

    if (err_check1 && !state) {
        ROS_WARN("[ ##### Car_Ctrl topic is not connected ##### ]");
        err_check1 = state;
    }
    else {
        err_check1 = state;
    }

    if (err_check2 && state) {
        ROS_INFO("[ ##### Car_Ctrl topic is connected ##### ]");
        err_check2 = !state;
    }
    else {
        err_check2 = !state;
    }

    return state;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Car_Plugin");
    ros::NodeHandle nh;
    // ros::Subscriber LinkState_Sub = nh.subscribe("/gazebo/link_states", 1, LinkState_CB);
    // ros::Subscriber ModelState_Sub = nh.subscribe("/gazebo/model_states", 1, ModelState_CB);
    // ros::Subscriber angle_Sub = nh.subscribe("/wheel_angle", 1, Angle_CB);
    // ros::Subscriber linear_Sub = nh.subscribe("/linear_speed", 1, Linear_CB);
    ros::Subscriber Ctrl_Sub = nh.subscribe("Car_Ctrl", 1, Ctrl_CB);

    ros::Publisher FR_Steer_Pub = nh.advertise<std_msgs::Float64>("/a_car/FR_steer_ctrl/command", 1);
    ros::Publisher FL_Steer_Pub = nh.advertise<std_msgs::Float64>("/a_car/FL_steer_ctrl/command", 1);
    ros::Publisher FR_Wheel_Pub = nh.advertise<std_msgs::Float64>("/a_car/FR_wheel_ctrl/command", 1);
    ros::Publisher FL_Wheel_Pub = nh.advertise<std_msgs::Float64>("/a_car/FL_wheel_ctrl/command", 1);
    ros::Publisher RR_Wheel_Pub = nh.advertise<std_msgs::Float64>("/a_car/RR_wheel_ctrl/command", 1);
    ros::Publisher RL_Wheel_Pub = nh.advertise<std_msgs::Float64>("/a_car/RL_wheel_ctrl/command", 1);
    ros::Publisher Wheel_Ang_Pub = nh.advertise<std_msgs::Float64>("/wheel_angle_data", 1);

    MyRobotHW robot;
    controller_manager::ControllerManager cm(&robot, nh);

    std_msgs::Float64 rstr;
    std_msgs::Float64 lstr;
    std_msgs::Float64 thr;
    std_msgs::Float64 ang;

    ros::Rate rate(30.0);

    while (ros::ok()) {
        bool Ctrl_Connected = Connection_Check(Ctrl_Sub);
        // bool Ctrl_Connected = Connection_Check(angle_Sub);

        std::tuple<float, float, float> ACM = ackermann_steer(Strf);
        // std::tuple<float, float, float> ACM = ackermann_steer(angle);
        float ACR_str = std::get<0>(ACM);
        float ACL_str = std::get<1>(ACM);
        float ACM_ang = std::get<2>(ACM);
        if (Ctrl_Connected) {
            rstr.data = ACR_str;
            lstr.data = ACL_str;
            thr.data = Thrf;
            // thr.data = linear;
            ang.data = ACM_ang;
            // ROS_INFO("Angle : %f", ang.data);
        }
        else {
            rstr.data = 0.0;
            lstr.data = 0.0;
            thr.data = 0.0;
            ang.data = 0;
        }

        FR_Steer_Pub.publish(rstr);
        FL_Steer_Pub.publish(lstr);
        FR_Wheel_Pub.publish(thr);
        FL_Wheel_Pub.publish(thr);
        RR_Wheel_Pub.publish(thr);
        RL_Wheel_Pub.publish(thr);
        Wheel_Ang_Pub.publish(ang);
        // ROS_INFO("Throttle: %f right: %f left: %f", thr.data, rstr.data, lstr.data);

        robot.read();
        cm.update(ros::Time::now(), ros::Duration(0.1));
        robot.write();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};