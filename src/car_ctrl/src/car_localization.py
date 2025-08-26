#!/usr/bin/env python3

import math
import rospy

import tf.transformations as transformations

from std_msgs.msg import Int16, Float64, Float64MultiArray


lz_gps_crd = [0, 0]

gps_speed = 0
encoder, encoder_1m = 0, 6.5
encoder_x, encoder_y, od_encoder = 0, 0, 0
current_speed, current_acceleration = 0, 0
encoder_current_time, encoder_last_time = 0, 0
def Encoder_CB(msg):
    global gps_speed
    global yaw, wheel_anlge
    global gps_crd, lz_gps_crd
    global encoder, delta_encoder
    global encoder_x, encoder_y, od_encoder
    global current_speed, current_acceleration
    global encoder_current_time, encoder_last_time

    encoder = msg.data

    encoder_current_time = float(rospy.Time.now().to_sec())
    delta_time = (encoder_current_time - encoder_last_time)
    delta_encoder = (encoder - od_encoder) / encoder_1m
    current_speed = (delta_encoder / delta_time)
    # current_acceleration = current_speed / delta_time

    if current_speed >= 0:
        en_eyaw = yaw - wheel_anlge
    else:
        en_eyaw = yaw + wheel_anlge

    encoder_x += delta_encoder * math.cos(en_eyaw)
    encoder_y += delta_encoder * math.sin(en_eyaw)

    lz_gps_crd[0] = gps_crd[0] + encoder_x
    lz_gps_crd[1] = gps_crd[1] + encoder_y

    lz_crd = Float64MultiArray(data=[lz_gps_crd[0], lz_gps_crd[1]])
    Localization_pub.publish(lz_crd)

    gps_speed = current_speed
    od_encoder = encoder
    encoder_last_time = encoder_current_time
    # rospy.sleep(0.1)

gps_crd = [0, 0]
def GPS_CB(msg):
    global gps_speed
    global delta_encoder
    global gps_crd, yaw_save
    global encoder_x, encoder_y, current_speed

    gps_crd[0] = msg.data[0] + gps_speed * math.cos(yaw)
    gps_crd[1] = msg.data[1] + gps_speed * math.sin(yaw)

    encoder_x = 0
    encoder_y = 0

wheel_anlge = 0
def Wheel_Angle_CB(msg):
    global wheel_anlge
    wheel_anlge = msg.data

yaw_save = []
roll, pitch, yaw = 0, 0, 0
ori_x, ori_y, ori_z, ori_w = 0, 0, 0, 0
roll_deg, pitch_deg, yaw_deg = 0, 0, 0
def IMU_CB(msg):
    global yaw_save
    global roll, pitch, yaw
    global ori_x, ori_y, ori_z, ori_w
    global roll_deg, pitch_deg, yaw_deg
    ori_x = msg.data[0]
    ori_y = msg.data[1]
    ori_z = msg.data[2]
    ori_w = msg.data[3]
    quaternion = [ori_x, ori_y, ori_z, ori_w]
    euler = transformations.euler_from_quaternion(quaternion)
    roll, pitch, yaw = euler
    yaw_save.append(yaw)

if __name__ == "__main__":
    rospy.init_node('localization', anonymous=False)

    Localization_pub = rospy.Publisher("lz_gps_crd", Float64MultiArray, queue_size=1)

    GPS_sub = rospy.Subscriber("/gps_crds", Float64MultiArray, GPS_CB, queue_size=1)
    IMU_sub = rospy.Subscriber("/imu_data", Float64MultiArray, IMU_CB, queue_size=1)
    Encoder_sub = rospy.Subscriber("/encoder_data", Int16, Encoder_CB, queue_size=1)
    Wheel_Angle_sub = rospy.Subscriber("/wheel_angle_data", Float64, Wheel_Angle_CB, queue_size=1)

    rate = rospy.Rate(30)

    try:
        # while (not rospy.is_shutdown()):
            # gps_crd[0] = gps_crd[0] + gps_speed * math.cos(yaw)
            # gps_crd[1] = gps_crd[1] + gps_speed * math.sin(yaw)
            # rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass