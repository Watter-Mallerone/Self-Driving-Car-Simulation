#! /usr/bin/env python3

# import math
import rospy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, Float32MultiArray, Float64MultiArray
from geometry_msgs.msg import Pose2D

gps_crds = [0]*2
yaw_data = [0]*4
# pose_msg = PoseStamped
def Bumblebee_Data_CB(msg : PoseStamped):
    pos_x, pos_y = msg.pose.position.x, msg.pose.position.y
    ori_x, ori_y, ori_z, ori_w = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
    gps_crds = Float64MultiArray(data=[pos_x, pos_y])
    yaw_data = Float64MultiArray(data=[ori_x, ori_y, ori_z, ori_w])
    GPS_pub.publish(gps_crds)
    IMU_pub.publish(yaw_data)

def a_car_ctrl_CB(msg : Float32MultiArray):
    throttle = int(msg.data[0])
    steering = int(msg.data[1])

    data = Pose2D()
    data.x = throttle * 2
    data.y = - steering

    rospy.loginfo(data)
    henes_pub.publish(data)

    # Bumblebee_wheel_angle = Int16(steering)
    # Bumblebee_linear_speed = Int16(throttle)
    # Bumblebee_wheel_angle_pub.publish(Bumblebee_wheel_angle)
    # Bumblebee_linear_speed_pub.publish(Bumblebee_linear_speed)

if __name__ == '__main__':
    rospy.init_node('data_converter', anonymous=False)

    GPS_pub = rospy.Publisher("/gps_crds", Float64MultiArray, queue_size=1)
    IMU_pub = rospy.Publisher("/imu_data", Float64MultiArray, queue_size=1)

    henes_pub = rospy.Publisher("/navigation_speed_and_angle", Pose2D, queue_size=1)
    # Bumblebee_wheel_angle_pub = rospy.Publisher("wheel_angle", Int16, queue_size=1)
    # Bumblebee_linear_speed_pub = rospy.Publisher("linear_speed", Int16, queue_size=1)

    a_car_ctrl_sub = rospy.Subscriber("/Car_Ctrl", Float32MultiArray, a_car_ctrl_CB, queue_size=1)
    Bumblebee_GPS_sub = rospy.Subscriber("/ekf_xy_yaw", PoseStamped, Bumblebee_Data_CB, queue_size=1)
    # Bumblebee_GPS_sub = rospy.Subscriber("/utm", PoseStamped, Bumblebee_Data_CB, queue_size=1)
    rospy.spin()