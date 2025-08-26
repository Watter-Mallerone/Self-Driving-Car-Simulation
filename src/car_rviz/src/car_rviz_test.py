#!/usr/bin/env python3

import math
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

gps_crd = [0, 0]
def GPS_CB(msg):
    global gps_crd
    gps_crd[0] = msg.data[0]
    gps_crd[1] = msg.data[1]
    gps_array.append(gps_crd)

lz_gps_crd = [0, 0]
def LZ_GPS_CB(msg):
    global lz_gps_crd
    lz_gps_crd[0] = msg.data[0]
    lz_gps_crd[1] = msg.data[1]
    lz_gps_array.append(lz_gps_crd)

gps_array = []
def GPS_Marker_Pub():
    global gps_array

    GPS_marker = Marker()
    GPS_marker_array = MarkerArray()

    GPS_marker.header.frame_id = "world"
    GPS_marker.header.stamp = rospy.Time.now()
    GPS_marker.ns = "point"
    GPS_marker.id = 0
    GPS_marker.type = Marker.POINTS
    GPS_marker.action = Marker.ADD
    GPS_marker.scale.x = 0.1
    GPS_marker.scale.y = 0.1
    GPS_marker.scale.z = 0.1

    GPS_marker.color.r = 1.0
    GPS_marker.color.g = 0.0
    GPS_marker.color.b = 0.0
    GPS_marker.color.a = 1.0

    GPS_marker.pose.orientation.w = 1

    if len(gps_array) > 0:
        for i in range(len(gps_array)):
            point = Point(gps_array[i][0], gps_array[i][1], 0.1)
            GPS_marker.points.append(point)

        GPS_marker_array.markers.append(GPS_marker)
        GPS_Marker_pub.publish(GPS_marker_array)

lz_gps_array = []
def LZ_GPS_Marker_Pub():
    global lz_gps_array

    LZ_GPS_marker = Marker()
    LZ_GPS_marker_array = MarkerArray()

    LZ_GPS_marker.header.frame_id = "world"
    LZ_GPS_marker.header.stamp = rospy.Time.now()
    LZ_GPS_marker.ns = "point"
    LZ_GPS_marker.id = 0
    LZ_GPS_marker.type = Marker.POINTS
    LZ_GPS_marker.action = Marker.ADD
    LZ_GPS_marker.scale.x = 0.2
    LZ_GPS_marker.scale.y = 0.2
    LZ_GPS_marker.scale.z = 0.2

    LZ_GPS_marker.color.r = 0.0
    LZ_GPS_marker.color.g = 1.0
    LZ_GPS_marker.color.b = 0.0
    LZ_GPS_marker.color.a = 1.0

    LZ_GPS_marker.pose.orientation.w = 1

    if len(lz_gps_array) > 0:
        for i in range(len(lz_gps_array)):
            point = Point(lz_gps_array[i][0], lz_gps_array[i][1], 0.1)
            LZ_GPS_marker.points.append(point)

        LZ_GPS_marker_array.markers.append(LZ_GPS_marker)
        LZ_GPS_Marker_pub.publish(LZ_GPS_marker_array)

if __name__ == '__main__':
    rospy.init_node('car_rviz', anonymous=False)

    GPS_Marker_pub = rospy.Publisher("/gps_marker", MarkerArray, queue_size=1)
    LZ_GPS_Marker_pub = rospy.Publisher("/lz_gps_marker", MarkerArray, queue_size=1)

    GPS_sub = rospy.Subscriber("/gps_crds", Float64MultiArray, GPS_CB, queue_size=1)
    LZ_GPS_sub = rospy.Subscriber("/lz_gps_crd", Float64MultiArray, LZ_GPS_CB, queue_size=1)

    rate = rospy.Rate(30)

    try:
        while(not rospy.is_shutdown()):
            if len(lz_gps_array) > 500:
                del lz_gps_array[0]
            GPS_Marker_Pub()
            LZ_GPS_Marker_Pub()
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
