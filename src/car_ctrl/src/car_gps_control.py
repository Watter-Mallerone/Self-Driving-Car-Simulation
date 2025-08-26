#!/usr/bin/env python3

import math
import yaml
import rospy

import tf.transformations as transformations

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray

def clamp(Num, Min, Max):
    return max(min(Num,Max),Min)

gps_x, gps_y = 0, 0
def GPS_CB(msg):
    global gps_x, gps_y
    gps_x = msg.data[0]
    gps_y = msg.data[1]
    # print(f'gps_x : {float(gps_x)} | gps_y : {float(gps_y)}')

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

path_x = []
def Path_X_CB(msg):
    global path_x
    path_x = msg.data

path_y = []
def Path_Y_CB(msg):
    global path_y
    path_y = msg.data

Line_GPS_X = []
def Line_GPS_X_CB(msg):
    global Line_GPS_X
    Line_GPS_X = msg.data

Line_GPS_Y = []
def Line_GPS_Y_CB(msg):
    global Line_GPS_Y
    Line_GPS_Y = msg.data

Lidar_GPS_X = []
def Lidar_GPS_X_CB(msg):
    global Lidar_GPS_X
    Lidar_GPS_X = msg.data

Lidar_GPS_Y = []
def Lidar_GPS_Y_CB(msg):
    global Lidar_GPS_Y
    Lidar_GPS_Y = msg.data

target = [0,0]

target_num = 0
distance = 99999

state = False
arrived = False
arrive_log = False
mode = None
def target_set():
    global distance
    global target_num
    global arrive_log, state, mode

    if len(waypoint) > 0:
        if target_num + 1 == len(waypoint):
            if distance < 1:
                target_num += 1
        elif target_num < len(waypoint):
            if distance < 3:
                target_num += 1

        if target_num < len(waypoint):
            arrive_log = False
            target[0] = waypoint[target_num][0]
            target[1] = waypoint[target_num][1]
            mode = waypoint[target_num][2]
            print(f"GPS point : {target_num}")
            state = False
        elif target_num == len(waypoint) and not arrive_log:
            rospy.loginfo("[ ##### Arrived ##### ]")
            arrive_log = True
            state = True
    elif state == False:
        rospy.logwarn("[ ##### Warning : NO GPS target data ##### ]")
        state = True

    return state

throttle = 0
def throttle_cal():
    global distance
    global track_x, track_y
    global throttle, steering

    if (track_x != None) and (track_y != None):
        distance = math.sqrt((target[0] - gps_x)**2 + (target[1] - gps_y)**2)
        path_distance = math.sqrt((track_x - gps_x)**2 + (track_y - gps_y)**2)
        if mode == "gps":
            throttle = clamp(distance ** 2, 0 ,10) / 10
        else:
            throttle = clamp(distance ** 2, 0 ,10) / 10

        if mode == "path":
            throttle = clamp(path_distance ** 2, 0 ,10) / 10
        print(f"distance : {distance}")
    else:
        throttle = 0

    throttle = throttle * 30

    throttle = throttle * (clamp(1 - (abs(steering) / 120), 0.2, 1))
    throttle = clamp(throttle, -10, 30)
    return throttle

steering = 0
def steering_cal():
    global yaw
    global track_x, track_y
    global throttle, steering

    if (track_x != None) and (track_y != None):
        rlt_x = track_x - gps_x
        rlt_y = track_y - gps_y

        rlt_ang = - math.degrees(math.atan2(rlt_y, rlt_x) - yaw)

        steering = rlt_ang
        if steering > 180:
            steering = steering - 360
        if steering < -180:
            steering = steering + 360
        # print(steering)
    else:
        steering = 0

    steering = clamp(steering, -30, 30)
    return steering

def GPS_Point_Pub():
    global track_x, track_y
    gps_goal_marker = Marker()
    gps_goal_array = MarkerArray()

    gps_goal_marker.header.frame_id = "world"
    gps_goal_marker.header.stamp = rospy.Time.now()
    gps_goal_marker.ns = "lines"
    gps_goal_marker.id = 1
    gps_goal_marker.type = Marker.LINE_STRIP
    gps_goal_marker.action = Marker.ADD
    gps_goal_marker.scale.x = 0.1
    gps_goal_marker.scale.y = 0.1
    gps_goal_marker.scale.z = 0.1

    gps_goal_marker.color.r = 0.0
    gps_goal_marker.color.g = 1.0
    gps_goal_marker.color.b = 0.0
    gps_goal_marker.color.a = 1.0

    gps_goal_marker.pose.orientation.w = 1

    for crds in waypoint:
        points = Point(crds[0] ,crds[1], 0.1)
        gps_goal_marker.points.append(points)

    gps_goal_array.markers.append(gps_goal_marker)
    GPS_Marker_pub.publish(gps_goal_array)

def GPS_Path_Maker_Pub():
    gps_path_marker = Marker()
    gps_path_array = MarkerArray()

    gps_path_marker.header.frame_id = "world"
    gps_path_marker.header.stamp = rospy.Time.now()
    gps_path_marker.ns = "point"
    gps_path_marker.id = 0
    gps_path_marker.type = Marker.POINTS
    gps_path_marker.action = Marker.ADD
    gps_path_marker.scale.x = 0.3
    gps_path_marker.scale.y = 0.3
    gps_path_marker.scale.z = 0.3

    gps_path_marker.pose.orientation.w = 1

    gps_path_marker.color.r = 0.0
    gps_path_marker.color.g = 0.0
    gps_path_marker.color.b = 1.0
    gps_path_marker.color.a = 1.0

    if len(path) > 0:
        for crds in path:
            points = Point(crds[0], crds[1], 0.1)
            gps_path_marker.points.append(points)

    gps_path_array.markers.append(gps_path_marker)
    GPS_Path_Markers_pub.publish(gps_path_array)

def Track_Marker_Pub():
    global track_x, track_y
    track_marker = Marker()
    track_array = MarkerArray()

    track_marker.header.frame_id = "world"
    track_marker.header.stamp = rospy.Time.now()
    track_marker.ns = "lines"
    track_marker.id = 1
    track_marker.type = Marker.LINE_STRIP
    track_marker.action = Marker.ADD
    track_marker.scale.x = 0.1
    track_marker.scale.y = 0.1
    track_marker.scale.z = 0.1

    track_marker.color.r = 1.0
    track_marker.color.g = 0.0
    track_marker.color.b = 1.0
    track_marker.color.a = 1.0

    track_marker.pose.orientation.w = 1

    if (track_x != None) and (track_y != None):
        point = Point(gps_x , gps_y, 0.1)
        track_marker.points.append(point)
        point = Point(track_x, track_y, 0.1)
        track_marker.points.append(point)

    track_array.markers.append(track_marker)
    Track_Marker_pub.publish(track_array)

def Danger_Marker_Pub():
    global danger_spot
    danger_marker = Marker()

    danger_marker.header.frame_id = "world"
    danger_marker.header.stamp = rospy.Time.now()
    danger_marker.ns = "point"
    danger_marker.id = 0
    danger_marker.type = Marker.POINTS
    danger_marker.action = Marker.ADD
    danger_marker.scale.x = 0.3
    danger_marker.scale.y = 0.3
    danger_marker.scale.z = 0.3

    danger_marker.color.r = 1.0
    danger_marker.color.g = 0.0
    danger_marker.color.b = 0.0
    danger_marker.color.a = 1.0

    danger_marker.pose.orientation.w = 1

    if len(danger_spot) > 0:
        point = Point(danger_spot[0] , danger_spot[1], 0.1)
        danger_marker.points.append(point)

    Danger_Marker_pub.publish(danger_marker)

car_ctrl = [0]*2
danger_spot = []
corrent_waypoint = [0]*2

thr = 0
str = 0
track_x = 0
track_y = 0

danger_rlt_dist = 99999
danger_rlt_ang = 0
danger_range = 0

wait = 0

width = 0.6
length = 1.3
command = "idle"

wall_reset = False
if __name__ == "__main__":
    rospy.init_node('gps_controller', anonymous=False)
    rospy.loginfo("[ ##### GPS Controller Online ##### ]")
    with open('/home/whstms1234/car_test/src/car_ctrl/config/car_gps_crds.yaml', 'r') as file:
        waypoint = yaml.full_load(file)

    gps_sub = rospy.Subscriber("/gps_crds", Float64MultiArray, GPS_CB, queue_size=1)
    imu_sub = rospy.Subscriber("/imu_data", Float64MultiArray, IMU_CB, queue_size=1)

    Path_X_sub = rospy.Subscriber("/path_x", Float64MultiArray, Path_X_CB, queue_size=1)
    Path_Y_sub = rospy.Subscriber("/path_y", Float64MultiArray, Path_Y_CB, queue_size=1)
    Line_GPS_X_sub = rospy.Subscriber("/line_gps_x", Float64MultiArray, Line_GPS_X_CB, queue_size=1)
    Line_GPS_Y_sub = rospy.Subscriber("/line_gps_y", Float64MultiArray, Line_GPS_Y_CB, queue_size=1)
    Lidar_GPS_X_sub = rospy.Subscriber("/lidar_gps_x", Float64MultiArray, Lidar_GPS_X_CB, queue_size=1)
    Lidar_GPS_Y_sub = rospy.Subscriber("/lidar_gps_y", Float64MultiArray, Lidar_GPS_Y_CB, queue_size=1)
    # LZ_GPS_sub = rospy.Subscriber("/lz_gps_crd", Float64MultiArray, GPS_CB, queue_size=1)

    Ctrl_pub = rospy.Publisher('Car_Ctrl', Float32MultiArray, queue_size = 1)
    corrent_waypoint_pub = rospy.Publisher('/corrent_waypoint', Float64MultiArray, queue_size = 1)
    reset_sig_pub = rospy.Publisher('/reset_signal', Bool, queue_size = 1)

    GPS_Marker_pub = rospy.Publisher('/gps_goal_marker', MarkerArray, queue_size = 1)
    GPS_Path_Markers_pub = rospy.Publisher('/gps_path_marker', MarkerArray, queue_size = 1)
    Track_Marker_pub = rospy.Publisher('/track_marker', MarkerArray, queue_size = 1)
    Danger_Marker_pub = rospy.Publisher('/danger_marker', Marker, queue_size = 1)

    rate = rospy.Rate(20)

    try:
        while(not rospy.is_shutdown()):
            line_wall = []
            line_wall = list(zip(Line_GPS_X, Line_GPS_Y))

            lidar_wall = []
            lidar_wall = list(zip(Lidar_GPS_X, Lidar_GPS_Y))

            dot_wall = []
            dot_wall = lidar_wall + line_wall

            path = []
            path = list(zip(path_x, path_y))

            if mode == "path":
                if len(path) > 0:
                    for i in range(len(path)):
                        track_rlt_x = path[i][0] - gps_x
                        track_rlt_y = path[i][1] - gps_y
                        track_rlt_dist = math.sqrt((track_rlt_x) ** 2 + (track_rlt_y) ** 2)
                        track_rlt_ang = - math.degrees(math.atan2(track_rlt_y, track_rlt_x) - yaw)

                        if track_rlt_ang > 180:
                            track_rlt_ang = track_rlt_ang - 360
                        if track_rlt_ang < -180:
                            track_rlt_ang = track_rlt_ang + 360

                        if len(path) > 100:
                            path = []
                            rospy.logwarn("Path length is too long.")
                            break

                        if ((track_rlt_dist > 2) and (track_rlt_dist < 5)) and ((track_rlt_ang < 90) and (track_rlt_ang > -90)):
                            track_x = path[i-1][0]
                            track_y = path[i-1][1]
                            break

            if mode == "gps":
                track_x = target[0]
                track_y = target[1]
            else:
                track_x = target[0]
                track_y = target[1]

            # if command != "stop":
            for i in range(len(dot_wall)):
                command = "idle"
                wall_reset = False
                wall_rlt_x = dot_wall[i][0] - gps_x
                wall_rlt_y = dot_wall[i][1] - gps_y
                wall_rlt_dist = math.sqrt((wall_rlt_x) ** 2 + (wall_rlt_y) ** 2)
                wall_rlt_ang = - math.degrees(math.atan2(wall_rlt_y, wall_rlt_x) - yaw)

                if wall_rlt_ang > 180:
                    wall_rlt_ang = wall_rlt_ang - 360
                if wall_rlt_ang < -180:
                    wall_rlt_ang = wall_rlt_ang + 360

                if wall_rlt_dist < length * 1.5 and (30 >= wall_rlt_ang and  wall_rlt_ang >= -30): # Front
                    print("Front_Stop")
                    command = "stop"
                    break

                if wall_rlt_dist < length/2 and (65 >= wall_rlt_ang and  wall_rlt_ang >= -65): # Front
                    print("Front_Stop")
                    command = "stop"
                    break
                if wall_rlt_dist < width/2 and (65 < wall_rlt_ang < 115): # Right
                    print("Right_Stop")
                    command = "stop"
                    break
                if wall_rlt_dist < width/2 and (-65 > wall_rlt_ang > -115): # Left
                    print("Left_Stop")
                    command = "stop"
                    break
                if wall_rlt_dist < length/2 and (115 <= wall_rlt_ang or wall_rlt_ang <= -115): # Rear
                    print("Rear_Stop")
                    command = "stop"
                    break

                if wall_rlt_dist < width * 2 and (70 < wall_rlt_ang < 120): # Right
                    print("Right_Dodge")
                    command = "dodge"
                    break
                if wall_rlt_dist < width * 2 and (-70 > wall_rlt_ang > -120): # Left
                    print("Left_Dodge")
                    command = "dodge"
                    break

                if wall_rlt_dist < 4 and (10 >= wall_rlt_ang and wall_rlt_ang >= -10): # Front
                    print("Front_Break")
                    command = "break"
                    break
                if wall_rlt_dist < 2 and (170 <= wall_rlt_ang or wall_rlt_ang <= -170): # Rear
                    print("Rear_Break")
                    command = "break"
                    break

                if wall_rlt_dist < 8 and (10 >= wall_rlt_ang and wall_rlt_ang >= -10): # Front
                    print("Front_Slow")
                    command = "slow"
                    break
                if wall_rlt_dist < 4 and (170 <= wall_rlt_ang or wall_rlt_ang <= -170): # Rear
                    print("Rear_Slow")
                    command = "slow"
                    break

            arrived = target_set()
            if not arrived:
                thr = throttle_cal()
                str = steering_cal()

                if command == "slow":
                    thr = throttle_cal() * 0.5
                    str = steering_cal()

                if command == "break":
                    thr = throttle_cal() * 0.25
                    str = steering_cal()

                if command == "dodge":
                    counter = steering_cal() * 1.5
                    thr = throttle_cal() * 0.6
                    str = steering_cal() + counter

                if command == "stop":
                    thr = 0
                    str = 0
                #     wall_reset = True
                #     wait += 1
                #     if wait >= 50:
                #         wall_reset = False
                #         command = "idle"
                #         wait = 0
                # else:
                #     wall_reset = False

            else:
                thr = 0
                str = 0
                mode == None
                command == None

            if mode == "path":
                corrent_waypoint = Float64MultiArray(data=[target[0], target[1]])
                corrent_waypoint_pub.publish(corrent_waypoint)
            else:
                corrent_waypoint = Float64MultiArray(data=[0.0, 0.0])
                corrent_waypoint_pub.publish(corrent_waypoint)

            if mode == "path":
                thr = throttle_cal() * 0.6
                str = steering_cal()

            if mode == "stop":
                wait += 1
                thr = 0
                str = 0
                if wait >= 50:
                    wait = 0
                    target_num = target_num + 1

            thr = clamp(thr ,-10, 30)
            str = clamp(str ,-30, 30)

            car_ctrl[0] = thr
            car_ctrl[1] = str
            ctrl_msg = Float32MultiArray(data=car_ctrl)

            # print(f"Mode : {mode}")
            # print(f"Reset : {wall_reset}")
            # print(f"Command : {command}")

            Ctrl_pub.publish(ctrl_msg)
            reset_sig_pub.publish(wall_reset)

            GPS_Point_Pub()
            GPS_Path_Maker_Pub()
            Track_Marker_Pub()
            Danger_Marker_Pub()
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        thr = 0
        str = 0
        Ctrl_pub.publish(ctrl_msg)
        rospy.logwarn("[ ##### GPS Controller Offline ##### ]")
        pass