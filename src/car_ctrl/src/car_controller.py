#! /usr/bin/env python3

import math
import yaml
import rospy
import sys, select, termios, tty, os
from std_msgs.msg import Float32MultiArray

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

def clamp(Num, Min, Max):
    return max(min(Num,Max),Min)

gps_x, gps_y = 0, 0
def GPS_CB(msg):
    global gps_x, gps_y
    gps_x = msg.data[0]
    gps_y = msg.data[1]

crds_save = []
from std_msgs.msg import Float64MultiArray
def get_GPS(key):
    global num
    gps_sub = rospy.Subscriber("/gps_crds", Float64MultiArray, GPS_CB, queue_size=1)

    if key == 'r':
        crds_save.append([gps_x, gps_y, len(crds_save) + 1])
        print(gps_x, gps_y)
        # print(len(crds_save))
        with open('/home/whstms1234/car_test/src/car_ctrl/config/car_get_gps_crds.yaml', 'w') as file: 
                yaml.dump(crds_save, file, default_flow_style=False, sort_keys=False)
        print("GPS_crds_saved")

throttle = 0
steering = 0
thr_v = 22
str_v = 3
car_ctrl =[0]*2
if __name__ == "__main__":
    rospy.init_node('car_controller',anonymous=False)
    rate = rospy.Rate(20)
    rospy.loginfo("[ ##### Controller_Online ##### ]")

    Ctrl_pub = rospy.Publisher('Car_Ctrl', Float32MultiArray, queue_size = 1)

    try:
        while(not rospy.is_shutdown()):
            press = getKey()
            if press == 'w':
                throttle = thr_v
            elif press == 's':
                throttle = -thr_v
            elif press == 'd':
                steering += str_v
            elif press == 'a':
                steering += -str_v
            elif press == 'f':
                throttle = 0
                steering = 0
            elif press == '':
                throttle = 0

            steering = clamp(steering, -30, 30)
            car_ctrl[0] = throttle
            car_ctrl[1] = steering
            # rospy.loginfo("pressed : %s \n", thr_press)
            # rospy.loginfo("pressed : %s \n", str_press)
            # rospy.loginfo("pressed : %d \n", throttle)
            # rospy.loginfo("pressed : %d \n", steering)

            msg = Float32MultiArray(data=car_ctrl)
            Ctrl_pub.publish(msg)
            get_GPS(press)
            rate.sleep()

    except rospy.ROSInterruptException:
        thr = 0
        sty = 0
        Ctrl_pub.publish(msg)
        rospy.loginfo("[ ##### Controller_Offline ##### ]")
        pass
