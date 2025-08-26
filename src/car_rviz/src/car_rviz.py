#!/usr/bin/env python3

import math
import yaml
import rospy

import tf.transformations as tf
import tf.transformations as transformations

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D

import sys, select, termios, tty, os

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
settings = termios.tcgetattr(sys.stdin)

crds_save = []
from std_msgs.msg import Float64MultiArray
def get_GPS(key):

    if key == 'r':
        with open('/home/whstms1234/car_test/src/car_ctrl/config/UTM1.yaml', 'w') as file: 
                yaml.dump(utm1, file, default_flow_style=False, sort_keys=False)
        print("UTM1_saved")

    if key == 't':
        with open('/home/whstms1234/car_test/src/car_ctrl/config/UTM2.yaml', 'w') as file: 
                yaml.dump(utm2, file, default_flow_style=False, sort_keys=False)
        print("UTM2_saved")

# def IMU_CB(msg) :
#     global roll, pitch, yaw

#     q = [msg.orientation.x, msg.orientation.y,msg.orientation.z, msg.orientation.w]
#     euler = tf.euler_from_quaternion(q)
#     roll, pitch, yaw = euler

gps_crds = [0]*2
yaw_data = [0]*4
roll, pitch, yaw = 0, 0, 0
ori_x, ori_y, ori_z, ori_w = 0, 0, 0, 0
roll_deg, pitch_deg, yaw_deg = 0, 0, 0
def Bumblebee_Data_CB(msg : PoseStamped):
    global roll, pitch, yaw
    global ori_x, ori_y, ori_z, ori_w
    global roll_deg, pitch_deg, yaw_deg

    pos_x, pos_y = msg.pose.position.x, msg.pose.position.y
    ori_x, ori_y, ori_z, ori_w = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
    gps_crds = [pos_x, pos_y]
    local_gps.append(gps_crds)

    yaw_data = [ori_x, ori_y, ori_z, ori_w]
    ori_x = yaw_data[0]
    ori_y = yaw_data[1]
    ori_z = yaw_data[2]
    ori_w = yaw_data[3]
    quaternion = [ori_x, ori_y, ori_z, ori_w]
    euler = transformations.euler_from_quaternion(quaternion)
    roll, pitch, yaw = euler
    roll_deg, pitch_deg, yaw_deg = roll * 180/math.pi, pitch * 180/math.pi, yaw * 180/math.pi

    rospy.sleep(0.01)

utm1 = []
def UTM1_CB(msg : PoseStamped):
    pos_x, pos_y = msg.pose.position.x, msg.pose.position.y
    gps_crds = [pos_x, pos_y]
    utm1.append(gps_crds)

utm2 = []
def UTM2_CB(msg : PoseStamped):
    pos_x, pos_y = msg.pose.position.x, msg.pose.position.y
    gps_crds = [pos_x, pos_y]
    utm2.append(gps_crds)

start = [0, 0]
def Start_CB(msg : Pose2D):
    global start
    pos_x, pos_y = msg.x, msg.y
    start = [pos_x, pos_y]

waypoint = []
def GPS_Get_Marker_Pub():
    GPS_get_marker = Marker()
    GPS_get_markerarray = MarkerArray()

    GPS_get_marker.header.frame_id = "map"
    GPS_get_marker.header.stamp = rospy.Time.now()
    GPS_get_marker.ns = "point"
    GPS_get_marker.id = 0
    GPS_get_marker.type = Marker.POINTS
    GPS_get_marker.action = Marker.ADD
    GPS_get_marker.scale.x = 0.3
    GPS_get_marker.scale.y = 0.3
    GPS_get_marker.scale.z = 0.3

    GPS_get_marker.color.r = 0.0
    GPS_get_marker.color.g = 1.0
    GPS_get_marker.color.b = 0.0
    GPS_get_marker.color.a = 1.0

    GPS_get_marker.pose.orientation.w = 1
    if len(waypoint) > 0:
        for crds in waypoint:
            points = Point(crds[0] - waypoint[0][0], crds[1] - waypoint[0][1], 0.1)
            GPS_get_marker.points.append(points)

        GPS_get_markerarray.markers.append(GPS_get_marker)
        GPS_Get_Marker_pub.publish(GPS_get_markerarray)
    elif len(waypoint) == None:
        print("None")

local_gps = []
def Local_GPS_Marker_Pub():
    global local_gps

    local_GPS_marker = Marker()
    local_GPS_markerarray = MarkerArray()

    local_GPS_marker.header.frame_id = "map"
    local_GPS_marker.header.stamp = rospy.Time.now()
    local_GPS_marker.ns = "point"
    local_GPS_marker.id = 0
    local_GPS_marker.type = Marker.POINTS
    local_GPS_marker.action = Marker.ADD
    local_GPS_marker.scale.x = 0.3
    local_GPS_marker.scale.y = 0.3
    local_GPS_marker.scale.z = 0.3

    local_GPS_marker.color.r = 1.0
    local_GPS_marker.color.g = 0.0
    local_GPS_marker.color.b = 1.0
    local_GPS_marker.color.a = 1.0

    local_GPS_marker.pose.orientation.w = 1

    if len(local_gps) > 0:
        for crds in local_gps:
            points = Point(crds[0] - start[0][0], crds[1] - start[0][1], 0.1)
            local_GPS_marker.points.append(points)

        local_GPS_markerarray.markers.append(local_GPS_marker)
        Local_GPS_Marker_pub.publish(local_GPS_markerarray)
    elif len(local_gps) == None:
        print("None")

def UTM1_Marker_Pub():
    global utm1

    UTM1_marker = Marker()
    UTM1_markerarray = MarkerArray()

    UTM1_marker.header.frame_id = "map"
    UTM1_marker.header.stamp = rospy.Time.now()
    UTM1_marker.ns = "point"
    UTM1_marker.id = 0
    UTM1_marker.type = Marker.POINTS
    UTM1_marker.action = Marker.ADD
    UTM1_marker.scale.x = 0.15
    UTM1_marker.scale.y = 0.15
    UTM1_marker.scale.z = 0.15

    UTM1_marker.color.r = 1.0
    UTM1_marker.color.g = 0.0
    UTM1_marker.color.b = 0.0
    UTM1_marker.color.a = 1.0

    UTM1_marker.pose.orientation.w = 1

    if len(start) > 0:
        for crds in utm1:
            points = Point(crds[0] - start[0], crds[1] - start[1], 0.1)
            UTM1_marker.points.append(points)

        UTM1_markerarray.markers.append(UTM1_marker)
        UTM1_pub.publish(UTM1_markerarray)
    elif len(utm1) == None:
        print("None")

def UTM2_Marker_Pub():
    global utm2

    UTM2_marker = Marker()
    UTM2_markerarray = MarkerArray()

    UTM2_marker.header.frame_id = "map"
    UTM2_marker.header.stamp = rospy.Time.now()
    UTM2_marker.ns = "point"
    UTM2_marker.id = 0
    UTM2_marker.type = Marker.POINTS
    UTM2_marker.action = Marker.ADD
    UTM2_marker.scale.x = 0.15
    UTM2_marker.scale.y = 0.15
    UTM2_marker.scale.z = 0.15

    UTM2_marker.color.r = 0.0
    UTM2_marker.color.g = 0.0
    UTM2_marker.color.b = 1.0
    UTM2_marker.color.a = 1.0

    UTM2_marker.pose.orientation.w = 1
    if len(utm2) > 0:
        for crds in utm2:
            points = Point(crds[0] - utm2[0][0], crds[1] - utm2[0][1], 0.1)
            UTM2_marker.points.append(points)

        UTM2_markerarray.markers.append(UTM2_marker)
        UTM2_pub.publish(UTM2_markerarray)
    elif len(utm2) == None:
        print("None")

utm1_saved = []
def UTM1_Save_Marker_Pub():
    global utm1_saved

    UTM1_Save_marker = Marker()
    UTM1_Save_markerarray = MarkerArray()

    UTM1_Save_marker.header.frame_id = "map"
    UTM1_Save_marker.header.stamp = rospy.Time.now()
    UTM1_Save_marker.ns = "point"
    UTM1_Save_marker.id = 0
    UTM1_Save_marker.type = Marker.POINTS
    UTM1_Save_marker.action = Marker.ADD
    UTM1_Save_marker.scale.x = 0.3
    UTM1_Save_marker.scale.y = 0.3
    UTM1_Save_marker.scale.z = 0.3

    UTM1_Save_marker.color.r = 1.0
    UTM1_Save_marker.color.g = 0.0
    UTM1_Save_marker.color.b = 0.0
    UTM1_Save_marker.color.a = 0.5

    UTM1_Save_marker.pose.orientation.w = 1
    if len(utm1_saved) > 0:
        for crds in utm1_saved:
            points = Point(crds[0] - utm1_saved[0][0], crds[1] - utm1_saved[0][1], 0.1)
            UTM1_Save_marker.points.append(points)
        UTM1_Save_markerarray.markers.append(UTM1_Save_marker)
        UTM1_Save_pub.publish(UTM1_Save_markerarray)
    elif len(utm1_saved) == None:
        print("None")

utm2_saved = []
def UTM2_Save_Marker_Pub():
    global utm2_saved

    UTM2_Save_marker = Marker()
    UTM2_Save_markerarray = MarkerArray()

    UTM2_Save_marker.header.frame_id = "map"
    UTM2_Save_marker.header.stamp = rospy.Time.now()
    UTM2_Save_marker.ns = "point"
    UTM2_Save_marker.id = 0
    UTM2_Save_marker.type = Marker.POINTS
    UTM2_Save_marker.action = Marker.ADD
    UTM2_Save_marker.scale.x = 0.3
    UTM2_Save_marker.scale.y = 0.3
    UTM2_Save_marker.scale.z = 0.3

    UTM2_Save_marker.color.r = 0.0
    UTM2_Save_marker.color.g = 0.0
    UTM2_Save_marker.color.b = 1.0
    UTM2_Save_marker.color.a = 0.5

    UTM2_Save_marker.pose.orientation.w = 1
    if len(utm2_saved) > 0:
        for crds in utm2_saved:
            points = Point(crds[0] - utm2_saved[0][0], crds[1] - utm2_saved[0][1], 0.1)
            UTM2_Save_marker.points.append(points)

        UTM2_Save_markerarray.markers.append(UTM2_Save_marker)
        UTM2_Save_pub.publish(UTM2_Save_markerarray)
    elif len(utm2_saved) == None:
        print("None")

def Yaw_pub():
    global local_gps

    Yaw_marker = Marker()

    Yaw_marker.header.frame_id = "map"
    Yaw_marker.header.stamp = rospy.Time.now()
    Yaw_marker.ns = "line"
    Yaw_marker.id = 0
    Yaw_marker.type = Marker.LINE_STRIP
    Yaw_marker.action = Marker.ADD
    Yaw_marker.scale.x = 0.3
    Yaw_marker.scale.y = 0.3
    Yaw_marker.scale.z = 0.3

    Yaw_marker.color.r = 1.0
    Yaw_marker.color.g = 1.0
    Yaw_marker.color.b = 1.0
    Yaw_marker.color.a = 1.0

    Yaw_marker.pose.orientation.w = 1
    if len(local_gps) > 0:
        for crds in local_gps:
            Yaw_marker.points = [
                                Point(crds[0] - local_gps[0][0], crds[1] - local_gps[0][1], 0.1),
                                Point((crds[0] - local_gps[0][0]) + 5 * math.cos(yaw), (crds[1] - local_gps[0][1]) + 5 * math.sin(yaw), 0.1)
                                ]
    Yaw_Marker_pub.publish(Yaw_marker)

if __name__ == '__main__':
    rospy.init_node('car_rviz', anonymous=False)

    Yaw_Marker_pub = rospy.Publisher('/local_yaw_marker', Marker, queue_size = 1)
    GPS_Get_Marker_pub = rospy.Publisher('/gps_get_marker', MarkerArray, queue_size = 1)
    Local_GPS_Marker_pub = rospy.Publisher('/local_gps_marker', MarkerArray, queue_size = 1)
    UTM1_pub = rospy.Publisher('/UTM1', MarkerArray, queue_size = 1)
    UTM2_pub = rospy.Publisher('/UTM2', MarkerArray, queue_size = 1)

    UTM1_Save_pub = rospy.Publisher('/UTM1_Saved', MarkerArray, queue_size = 1)
    UTM2_Save_pub = rospy.Publisher('/UTM2_Saved', MarkerArray, queue_size = 1)

    # Bumblebee_GPS_sub = rospy.Subscriber("/ekf_xy_yaw", PoseStamped, Bumblebee_Data_CB, queue_size=1)
    # Bumblebee_GPS_sub = rospy.Subscriber("/utm", PoseStamped, Bumblebee_Data_CB, queue_size=1)

    UTM1_sub = rospy.Subscriber("/ekf_xy_yaw", PoseStamped, UTM1_CB, queue_size=1)
    # UTM1_sub = rospy.Subscriber("/utm", PoseStamped, UTM1_CB, queue_size=1)
    # UTM2_sub = rospy.Subscriber("/utm2", PoseStamped, UTM2_CB, queue_size=1)

    # IMU_sub = rospy.Subscriber("/handsfree/imu", Imu, IMU_CB, queue_size=1)

    start_sub = rospy.Subscriber("/start_gps", Pose2D, Start_CB,queue_size=1)

    with open('/home/whstms1234/car_test/src/car_ctrl/config/car_get_gps_crds.yaml', 'r') as file:
        waypoint = yaml.full_load(file)

    # with open('/home/whstms1234/car_test/src/car_ctrl/config/UTM1.yaml', 'r') as file:
    #     utm1_saved = yaml.full_load(file)

    # with open('/home/whstms1234/car_test/src/car_ctrl/config/UTM2.yaml', 'r') as file:
    #     utm2_saved = yaml.full_load(file)

    rate = rospy.Rate(30)

    try:
        while(not rospy.is_shutdown()):
            # press = getKey()
            # get_GPS(press)

            # with open('/home/whstms1234/car_test/src/car_ctrl/config/car_get_gps_crds.yaml', 'r') as file:
            #     waypoint = yaml.full_load(file)

            # with open('/home/whstms1234/car_test/src/car_ctrl/config/UTM1.yaml', 'r') as file:
            #     utm1_saved = yaml.full_load(file)

            # with open('/home/whstms1234/car_test/src/car_ctrl/config/UTM2.yaml', 'r') as file:
            #     utm2_saved = yaml.full_load(file)

            if len(local_gps) > 1000:
                del local_gps[0]

            if len(utm1) > 1000:
                del utm1[0]
            if len(utm2) > 1000:
                del utm2[0]

            Yaw_pub()
            UTM1_Marker_Pub()
            UTM2_Marker_Pub()
            UTM1_Save_Marker_Pub()
            UTM2_Save_Marker_Pub()
            GPS_Get_Marker_Pub()
            Local_GPS_Marker_Pub()
            # rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
