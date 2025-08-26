#!/usr/bin/env python3

import math
import rospy

from std_msgs.msg import Bool, Float64MultiArray
import tf.transformations as transformations

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

gps_x, gps_y = 0, 0
def GPS_CB(msg):
    global gps_x, gps_y
    gps_x = msg.data[0]
    gps_y = msg.data[1]

roll, pitch, yaw = 0, 0, 0
ori_x, ori_y, ori_z, ori_w = 0, 0, 0, 0
roll_deg, pitch_deg, yaw_deg = 0, 0, 0
def IMU_CB(msg):
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
    roll_deg, pitch_deg, yaw_deg = roll * 180/math.pi, pitch * 180/math.pi, yaw * 180/math.pi

reset = 0
rlt_ang = 0
rlt_dist = 0
def Lidar_CB(data):
    global gps_x, gps_y
    global rlt_ang, rlt_dist
    global reset, rate

    distance, angle = 0, 0
    rlt_ang, rlt_dist = 0, 0
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges

    for i, distance in enumerate(ranges):
        if (distance < data.range_max) and (distance > 1.5):
            angle = -(angle_min + i * angle_increment)
            angle_deg = math.degrees(angle)
            if angle_deg < 45 and angle_deg > -45:
                rlt_dist = distance
                rlt_ang = angle
                # print(math.degrees(rlt_ang))
                Get_GPS()
                GPS_Marker_Pub()

    if len(ld_global_GPS) > 0:
        for i in range(len(ld_global_GPS)):
            ldc_x = ld_global_GPS[i][0]
            ldc_y = ld_global_GPS[i][1]
            ldc_rlt_x = ldc_x - gps_x
            ldc_rlt_y = ldc_y - gps_y
            ldc_dist = math.sqrt((ldc_rlt_x) ** 2 + (ldc_rlt_y) ** 2)
            ldc_rlt_ang = math.degrees(math.atan2(ldc_rlt_y, ldc_rlt_x) - yaw)

            if ldc_rlt_ang > 180:
                ldc_rlt_ang = ldc_rlt_ang - 360
            if ldc_rlt_ang < -180:
                ldc_rlt_ang = ldc_rlt_ang + 360
            if (ldc_dist > 15): #or not ((ldc_rlt_ang < 135) and (ldc_rlt_ang > -135)):
                del ld_global_GPS[i]
                break
    # reset += 1
    # rate.sleep()

ld_global_GPS = []
def Get_GPS():
    ld_len = len(ld_global_GPS)
    if (not(rlt_ang == 0)) and (not(rlt_dist == 0)):
        ld_gps_cal_x = gps_x + (rlt_dist + 0.3) * math.cos(yaw - rlt_ang)
        ld_gps_cal_y = gps_y + (rlt_dist + 0.3) * math.sin(yaw - rlt_ang)
        ld_gps_point = [ld_gps_cal_x, ld_gps_cal_y]

        if ld_len > 0:
            should_append1 = True
            for point in ld_global_GPS:
                if math.sqrt((ld_gps_point[0] - point[0]) ** 2 + (ld_gps_point[1] - point[1]) ** 2) < 0.3:
                    should_append1 = False
                    break
            if should_append1:
                ld_global_GPS.append(ld_gps_point)
        else:
            ld_global_GPS.append(ld_gps_point)

reset_signal = False
def Reset_CB(msg):
    global reset_signal
    reset_signal = msg.data

def GPS_Marker_Pub():
    ld_gps_marker = Marker()
    ld_gps_array = MarkerArray()

    ld_gps_marker.header.frame_id = "world"
    ld_gps_marker.header.stamp = rospy.Time.now()
    ld_gps_marker.ns = "point"
    ld_gps_marker.id = 0
    ld_gps_marker.type = Marker.POINTS
    ld_gps_marker.action = Marker.ADD
    ld_gps_marker.scale.x = 0.1
    ld_gps_marker.scale.y = 0.1
    ld_gps_marker.scale.z = 0.1

    ld_gps_marker.pose.orientation.w = 1

    ld_gps_marker.color.r = 0.0
    ld_gps_marker.color.g = 0.0
    ld_gps_marker.color.b = 1.0
    ld_gps_marker.color.a = 1.0

    for xy_point in ld_global_GPS:
        point = Point(xy_point[0], xy_point[1], 0.1)
        ld_gps_marker.points.append(point)

    ld_gps_array.markers.append(ld_gps_marker)
    LD_GPS_Marker_pub.publish(ld_gps_array)

if __name__ == '__main__':
    rospy.init_node('lidar', anonymous=False)

    LD_GPS_Marker_pub = rospy.Publisher("/lidar_gps_marker", MarkerArray, queue_size=1)
    Lidar_GPS_X_pub = rospy.Publisher("/lidar_gps_x", Float64MultiArray, queue_size=1)
    Lidar_GPS_Y_pub = rospy.Publisher("/lidar_gps_y", Float64MultiArray, queue_size=1)

    gps_sub = rospy.Subscriber("/gps_crds", Float64MultiArray, GPS_CB, queue_size=1)
    imu_sub = rospy.Subscriber("/imu_data", Float64MultiArray, IMU_CB, queue_size=1)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, Lidar_CB, queue_size=1)
    reset_sig_sub = rospy.Subscriber('/reset_signal', Bool, Reset_CB, queue_size=1)

    rospy.loginfo("[ ##### Lidar_Online ##### ]")

    rate = rospy.Rate(30)

    try:
        while(not rospy.is_shutdown()):
            # print(reset)
            if reset >= 100 or reset_signal:
                ld_global_GPS = []
                reset =0

            lidar_x = Float64MultiArray()
            lidar_y = Float64MultiArray()

            if len(ld_global_GPS) > 0:
                for crd in ld_global_GPS:
                    lidar_x.data.append(crd[0])
                    lidar_y.data.append(crd[1])

            Lidar_GPS_X_pub.publish(lidar_x)
            Lidar_GPS_Y_pub.publish(lidar_y)
            GPS_Marker_Pub()

            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("[ ##### Lidar_Offline ##### ]")
        pass
