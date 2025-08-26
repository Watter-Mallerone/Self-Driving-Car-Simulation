#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import math

from std_msgs.msg import Bool, Float64, Float64MultiArray, Int64MultiArray

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from cv_bridge import CvBridge, CvBridgeError

import threading
from visualization_msgs.msg import Marker, MarkerArray

import tf.transformations as transformations

#---- Tunning UI ----#
import os
import sys
import tkinter as tk
from tkinter import ttk
sys.path.append("..")
curr_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(curr_path)

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

reset_signal = False
def Reset_CB(msg):
    global reset_signal
    reset_signal = msg.data

class LineDetection:
    def __init__(self):
        self.converter = CvBridge()
        # self.Image_sub = rospy.Subscriber("/depth_cam/image_raw", Image, self.Img_to_CV)
        self.Image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.Img_to_CV)
        # self.Image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.Img_to_CV)
        self.Top = 200
        self.Bottom = 600
        self.Lenth = 200
        self.A_lower = 0
        self.B_lower = 0
        self.C_lower = 0
        self.A_upper = 255
        self.B_upper = 255
        self.C_upper = 255

    def Img_to_CV(self, data):
        try:
            # np_arr = np.frombuffer(data.data, np.uint8)
            # vid = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            vid = self.converter.imgmsg_to_cv2(data, "bgr8")
            # vid = self.converter.compressed_imgmsg_to_cv2(data, "bgr8")
            self.Line_Detection(vid)
        except CvBridgeError as converter_error:
            rospy.logerr(converter_error)
            return

    def CV_to_Img(self, data):
        try:
            processed_image = CvBridge().cv2_to_imgmsg(data, "bgr8")
            Image_pub.publish(processed_image)
        except CvBridgeError as converter_error:
            rospy.logerr(converter_error)
            return

##### ------------------------- Bird Eye View ---------------------------- #####
    def Line_Detection(self, vid):
        height, width = vid.shape[:2]
###       -------------- Tunning Setup --------------       ###
        # src = np.float32([
        #     [(width / 2) - self.Top, self.Lenth],   # Left - Top
        #     [(width / 2) + self.Top, self.Lenth],   # Right - Top
        #     [(width / 2) + self.Bottom, height],      # Right - Bottom
        #     [(width / 2) - self.Bottom, height]       # Left - Bottom
        # ])

###       -------------- Gazebo Setup --------------       ###
        # src = np.float32([
        #     [(width / 2) - 240, 200],         # Left - Top
        #     [(width / 2) + 240, 200],         # Right - Top
        #     [(width / 2) + 1300, height],                  # Right - Bottom
        #     [(width / 2) - 1300, height]                   # Left - Bottom
        # ])

###       -------------- Real Setup --------------       ###
        src = np.float32([
            [(width / 2) - 140, 150],         # Left - Top
            [(width / 2) + 140, 150],         # Right - Top
            [(width / 2) + 1000, height],                  # Right - Bottom
            [(width / 2) - 1000, height]                   # Left - Bottom
        ])

        dst = np.float32([
            [0.0, 0.0],                                # Left - Top
            [width, 0.0],                              # Right - Top
            [width, height],                           # Right - Bottom
            [0, height]                                # Left - Bottom
        ])

        mat = cv2.getPerspectiveTransform(src, dst)
        Bird_Eye_View = cv2.warpPerspective(vid, mat, (width, height))

        src = src.astype(int)
        cv2.polylines(vid, [src], isClosed=True, color=(0, 0, 255), thickness=1)
        cv2.imshow("Tunning_Image", vid) # <------

##### ------------------------- Line Detection ---------------------------- #####
        rgb = Bird_Eye_View

###       -------------- Real Setup --------------       ###
        rgb_white_lower = np.array([200, 200, 210])
        rgb_white_upper = np.array([255, 255, 255])
        rgb_yellow_lower = np.array([0, 0, 0])
        rgb_yellow_upper = np.array([0, 0, 0])

###       -------------- Gazebo Setup --------------       ###
        # rgb_white_lower = np.array([100, 100, 100])
        # rgb_white_upper = np.array([255, 255, 255])
        # rgb_yellow_lower = np.array([0, 0, 0])
        # rgb_yellow_upper = np.array([0, 0, 0])

        # rgb_white_lower = np.array([self.A_lower, self.B_lower, self.C_lower])
        # rgb_white_upper = np.array([self.A_upper, self.B_upper, self.C_upper])
        # rgb_yellow_lower = np.array([0, 0, 0])
        # rgb_yellow_upper = np.array([0, 0, 0])

        rgb_white_mask = cv2.inRange(rgb, rgb_white_lower, rgb_white_upper)
        rgb_yellow_mask = cv2.inRange(rgb, rgb_yellow_lower, rgb_yellow_upper)

        RGB_mask = cv2.bitwise_or(rgb_white_mask, rgb_yellow_mask)
        RGB_masked = cv2.bitwise_and(Bird_Eye_View, Bird_Eye_View, mask = RGB_mask)

        hls = cv2.cvtColor(Bird_Eye_View, cv2.COLOR_RGB2HLS)
        hls_white_lower = np.array([41, 195, 15])
        hls_white_upper = np.array([125, 255, 85])
        hls_yellow_lower = np.array([0, 0, 0])
        hls_yellow_upper = np.array([0, 0, 0])

        # hls_white_lower = np.array([self.A_lower, self.B_lower, self.C_lower])
        # hls_white_upper = np.array([self.A_upper, self.B_upper, self.C_upper])
        # hls_yellow_lower = np.array([0, 0, 0])
        # hls_yellow_upper = np.array([0, 0, 0])

        hls_white_mask = cv2.inRange(hls, hls_white_lower, hls_white_upper)
        hls_yellow_mask = cv2.inRange(hls, hls_yellow_lower, hls_yellow_upper)

        HLS_mask = cv2.bitwise_or(hls_white_mask, hls_yellow_mask)
        HLS_masked = cv2.bitwise_and(Bird_Eye_View, Bird_Eye_View, mask = HLS_mask)

        lab = cv2.cvtColor(Bird_Eye_View, cv2.COLOR_RGB2LAB)
        lab_white_lower = np.array([210, 109, 0])
        lab_white_upper = np.array([255, 135, 158])
        lab_yellow_lower = np.array([0, 0, 0])
        lab_yellow_upper = np.array([0, 0, 0])
    
        # lab_white_lower = np.array([self.A_lower, self.B_lower, self.C_lower])
        # lab_white_upper = np.array([self.A_upper, self.B_upper, self.C_upper])
        # lab_yellow_lower = np.array([0, 0, 0])
        # lab_yellow_upper = np.array([0, 0, 0])

        lab_white_mask = cv2.inRange(lab, lab_white_lower, lab_white_upper)
        lab_yellow_mask = cv2.inRange(lab, lab_yellow_lower, lab_yellow_upper)

        LAB_mask = cv2.bitwise_or(lab_white_mask, lab_yellow_mask)
        LAB_masked = cv2.bitwise_and(Bird_Eye_View, Bird_Eye_View, mask = LAB_mask)

        # ALL_mask = cv2.bitwise_or(RGB_mask, HLS_mask)
        # ALL_masked = cv2.bitwise_and(RGB_masked, HLS_masked, mask = ALL_mask)
        # ALL_mask = cv2.bitwise_or(cv2.bitwise_or(RGB_mask, LAB_mask), HLS_mask)
        # ALL_masked = cv2.bitwise_and(cv2.bitwise_and(RGB_masked, LAB_masked), HLS_masked, mask=ALL_mask)
        # added_vid1 = cv2.add(RGB_masked, HLS_masked)
        added_vid2 = cv2.add(RGB_masked, LAB_masked)
        # added_vid3 = cv2.add(added_vid1, added_vid2)

###       -------------- Color Filter Tunning --------------       ###
        # [self.A_lower, self.B_lower, self.C_lower]
        # [self.A_upper, self.B_upper, self.C_upper]
        # cv2.imshow("Raw_Image", vid)
        # cv2.imshow("rgb", rgb)
        # cv2.imshow("rgb_white_mask", rgb_white_mask)
        # cv2.imshow("rgb_yellow_mask", rgb_yellow_mask)
        # cv2.imshow("RGB_mask", RGB_mask)
        # cv2.imshow("RGB_masked", RGB_masked)
        # cv2.imshow("hls", hls)
        # cv2.imshow("hls_white_mask", hls_white_mask)
        # cv2.imshow("hls_yellow_mask", hls_yellow_mask)
        # cv2.imshow("HLS_mask", HLS_mask)
        # cv2.imshow("HLS_masked", HLS_masked)
        # cv2.imshow("lab", lab)
        # cv2.imshow("lab_white_mask", lab_white_mask)
        # cv2.imshow("lab_yellow_mask", lab_yellow_mask)
        # cv2.imshow("LAB_mask", LAB_mask)
        # cv2.imshow("LAB_masked", LAB_masked)
        # cv2.imshow("ALL_mask", ALL_mask)
        # cv2.imshow("ALL_masked", ALL_masked)
        # cv2.imshow("added_vid1", added_vid1)
        # cv2.imshow("added_vid2", added_vid2)
        # cv2.imshow("added_vid3", added_vid3)

##### ------------------------- Fill Color ---------------------------- #####
        # road_color = cv2.cvtColor(ALL_masked, cv2.COLOR_BGR2GRAY)
        # _, road_mask = cv2.threshold(road_color, 1, 255, cv2.THRESH_BINARY)
        # road_color_img = np.full_like(Bird_Eye_View, (0, 0, 255))  
        # Bird_Eye_View_colored = cv2.addWeighted(road_color_img, 0.8, Bird_Eye_View, 0.5, 0)
        # # cv2.imshow("Bird_Eye_View_colored", Bird_Eye_View_colored)
        # Bird_Eye_View_colored = cv2.bitwise_and(Bird_Eye_View_colored, Bird_Eye_View_colored, mask=road_mask)
        # # cv2.imshow("road_color_img", road_color_img)
        # cv2.imshow("Bird_Eye_View_colored", Bird_Eye_View_colored)

##### ------------------------- Line Detection ---------------------------- #####
        gray = cv2.cvtColor(added_vid2, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # cv2.imshow("gray", gray) # <------
        # cv2.imshow("blur", blur) # <------
        # cv2.imshow("edges", edges) # <------

        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (width, height),
            (width, int(height // 5)-1),
            (0, int(height // 5)-1),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        # cv2.imshow("cropped_edges", cropped_edges) # <------
    
        def cross_filter(line1, line2):
            def ccw(A, B, C):
                return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

            A = (line1[0][0], line1[0][1])
            B = (line1[0][2], line1[0][3])
            C = (line2[0][0], line2[0][1])
            D = (line2[0][2], line2[0][3])

            return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

        def gap_filter(lines, min_dist):
            if lines is None:
                return []
            filtered_lines = []
            for i, line1 in enumerate(lines):
                lx1, ly1, lx2, ly2 = line1[0]
                should_add = True
                for j, line2 in enumerate(filtered_lines):
                    if i == j:
                        continue
                    dx1, dy1, dx2, dy2 = line2[0]
                    dist = min(
                        np.sqrt((lx1 - dx1) ** 2 + (ly1 - dy1) ** 2),
                        np.sqrt((lx1 - dx2) ** 2 + (ly1 - dy2) ** 2),
                        np.sqrt((lx2 - dx1) ** 2 + (ly2 - dy1) ** 2),
                        np.sqrt((lx2 - dx2) ** 2 + (ly2 - dy2) ** 2)
                    )
                    if dist < min_dist or cross_filter(line1, line2):
                        should_add = False
                        break

                if should_add:
                    filtered_lines.append(line1)
            return filtered_lines

        lines = cv2.HoughLinesP(cropped_edges, 2, np.pi/180, 50, maxLineGap=50)
        lines = gap_filter(lines, 10)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(Bird_Eye_View, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            rospy.logwarn("[ ##### No lines detected ##### ]")
            return

        self.CV_to_Img(Bird_Eye_View)
        # cv2.imshow("Raw_Image", vid)
        cv2.imshow("Prosecced_Image", Bird_Eye_View)
        # cv2.imshow("Fill_Color_Image", Bird_Eye_View_colored)
        Line_Marker_Pub(lines)
        Get_Line_GPS_Point(lines)
        Global_Line_Point_Pub()
        cv2.waitKey(1)

def Delete_Marker():
    delete_marker = Marker()
    delete_marker.action = Marker.DELETEALL
    return delete_marker

def Line_Marker_Pub(lines):
    marker_id = 0
    marker = Marker()
    marker_array = MarkerArray()

    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lines"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.02

    marker.pose.orientation.w = 1

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    for line in lines:
###       -------------- Gazebo Setup --------------       ###
        # x1, y1 = line[0][0]/85, line[0][1]/60
        # x2, y2 = line[0][2]/85, line[0][3]/60

        # marker.points = [
        #     Point(-y1 + 8.8, -x1 + 3.6, -0.15),
        #     Point(-y2 + 8.8, -x2 + 3.6, -0.15)
        # ]

##       -------------- Real Setup --------------       ###
        x1, y1 = line[0][0]/480, line[0][1]/480
        x2, y2 = line[0][2]/480, line[0][3]/480

        marker.points = [
            Point(-y1 + 4.5, -x1 + 2.0, -0.15),
            Point(-y2 + 4.5, -x2 + 2.0, -0.15)
        ]

        marker_array.markers.append(marker)
        marker_id += 1

    Marker_pub.publish(marker_array)

global_line_points = []
def Get_Line_GPS_Point(lines):
    gl_dist = 0
    gl_rlt_ang = 0

###       -------------- Gazebo Setup --------------       ###
    # for line in lines:
    #     gl_x1, gl_y1 = (line[0][0] / 640 - 0.5) * 3.7, ((480 - line[0][1]) / 480 * 3.9) + 0.5
    #     gl_rlt_ang = math.atan2(gl_x1, gl_y1)
    #     gl_dist = math.sqrt(gl_x1**2 + gl_y1**2) * 2

###       -------------- Real Setup --------------       ###
    for line in lines:
        gl_x1, gl_y1 = (line[0][0] / 640 - 0.5) * 3.7, ((480 - line[0][1]) / 480 * 3.9) + 0.5
        gl_rlt_ang = math.atan2(gl_x1, gl_y1)
        gl_dist = math.sqrt(gl_x1**2 + gl_y1**2) * 2

    global_line_point_cal_x = gps_x + gl_dist * math.sin(-yaw + gl_rlt_ang + math.pi / 2)
    global_line_point_cal_y = gps_y + gl_dist * math.cos(-yaw + gl_rlt_ang + math.pi / 2)
    global_line_point_cal = [float(global_line_point_cal_x), float(global_line_point_cal_y)]

    gl_len = len(global_line_points)
    if gl_len > 0:
        should_append1 = True
        for point in global_line_points:
                point_dist = math.sqrt((global_line_point_cal[0] - point[0]) ** 2 + (global_line_point_cal[1] - point[1]) ** 2)
                point_rlt_dist = math.sqrt((global_line_point_cal[0] - gps_x) ** 2 + (global_line_point_cal[1] - gps_y) ** 2)
                if point_rlt_dist < 1 or point_dist < 0.3:
                    should_append1 = False
                    break
        if should_append1:
            global_line_points.append(global_line_point_cal)

        for i in range(gl_len):
            glc_x = global_line_points[i][0]
            glc_y = global_line_points[i][1]
            glc_rlt_x = glc_x - gps_x
            glc_rlt_y = glc_y - gps_y

            glc_rlt_ang = math.degrees(math.atan2(glc_rlt_y, glc_rlt_x) - yaw)
            glc_dist = math.sqrt((glc_rlt_x) ** 2 + (glc_rlt_y) ** 2)

            if glc_rlt_ang > 180:
                glc_rlt_ang = glc_rlt_ang - 360
            if glc_rlt_ang < -180:
                glc_rlt_ang = glc_rlt_ang + 360

            if (glc_dist > 15): #or not ((glc_rlt_ang < 170) and (glc_rlt_ang > -170)):
                del global_line_points[i]
                break            
    else:
        global_line_points.append(global_line_point_cal)

def Global_Line_Point_Pub():
    global_line_marker = Marker()
    global_line_array = MarkerArray()

    # delete_marker = Marker()
    # delete_marker.action = Marker.DELETE
    # global_line_array.markers.append(delete_marker)

    global_line_marker.header.frame_id = "world"
    global_line_marker.header.stamp = rospy.Time.now()
    global_line_marker.ns = "point"
    global_line_marker.id = 0
    global_line_marker.type = Marker.POINTS
    global_line_marker.action = Marker.ADD
    global_line_marker.scale.x = 0.1
    global_line_marker.scale.y = 0.1
    global_line_marker.scale.z = 0.1

    global_line_marker.pose.orientation.w = 1

    global_line_marker.color.r = 0.0
    global_line_marker.color.g = 1.0
    global_line_marker.color.b = 0.0
    global_line_marker.color.a = 1.0

    for crds in global_line_points:
        points = Point(crds[0] ,crds[1], 0.1)
        global_line_marker.points.append(points)

    global_line_array.markers.append(global_line_marker)
    Global_Line_Marker_pub.publish(global_line_array)

def frame_tunning(ld):
    def Top(val):
        ld.Top = int(val)
        # print(f"Top : {ld.Top}")

    def Bottom(val):
        ld.Bottom = int(val)
        # print(f"Bottom : {ld.Bottom}")

    def Lenth(val):
        ld.Lenth = int(val)
        # print(f"Lenth : {ld.Bottom}")

    window = tk.Tk()
    window.title("frame_tunning")

    tk.Label(window, text="Top Parameter").pack()
    tilting_slider = tk.Scale(window, from_=0, to=1000, orient=tk.HORIZONTAL, command=Top)
    tilting_slider.set(ld.Top)
    tilting_slider.pack()

    tk.Label(window, text="Bottom Parameter").pack()
    z_slider = tk.Scale(window, from_=0, to=1400, orient=tk.HORIZONTAL, command=Bottom)
    z_slider.set(ld.Bottom)
    z_slider.pack()

    tk.Label(window, text="Lenth Parameter").pack()
    z_slider = tk.Scale(window, from_=0, to=1000, orient=tk.HORIZONTAL, command=Lenth)
    z_slider.set(ld.Lenth)
    z_slider.pack()

    window.mainloop()

def color_tunning(ld):
    def A_lower(val):
        ld.A_lower = int(val)
    def B_lower(val):
        ld.B_lower = int(val)
    def C_lower(val):
        ld.C_lower = int(val)
    def A_upper(val):
        ld.A_upper = int(val)
    def B_upper(val):
        ld.B_upper = int(val)
    def C_upper(val):
        ld.C_upper = int(val)

    window = tk.Tk()
    window.title("Color Tunning")

    tk.Label(window, text="A_lower").pack()
    z_slider = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL, command=A_lower)
    z_slider.set(ld.A_lower)
    z_slider.pack()
    tk.Label(window, text="B_lower").pack()
    z_slider = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL, command=B_lower)
    z_slider.set(ld.B_lower)
    z_slider.pack()
    tk.Label(window, text="C_lower").pack()
    z_slider = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL, command=C_lower)
    z_slider.set(ld.C_lower)
    z_slider.pack()
    tk.Label(window, text="A_upper").pack()
    z_slider = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL, command=A_upper)
    z_slider.set(ld.A_upper)
    z_slider.pack()
    tk.Label(window, text="B_upper").pack()
    z_slider = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL, command=B_upper)
    z_slider.set(ld.B_upper)
    z_slider.pack()
    tk.Label(window, text="C_upper").pack()
    z_slider = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL, command=C_upper)
    z_slider.set(ld.C_upper)
    z_slider.pack()

    window.mainloop()
    
if __name__ == '__main__':
    rospy.init_node('line_detection', anonymous=False)
    Image_pub = rospy.Publisher("/line_image", Image, queue_size=1)
    Marker_pub = rospy.Publisher("/line_marker_array", MarkerArray, queue_size=1)
    Global_Line_Marker_pub = rospy.Publisher("/global_line_array", MarkerArray, queue_size=1)
    Line_GPS_X_pub = rospy.Publisher("/line_gps_x", Float64MultiArray, queue_size=1)
    Line_GPS_Y_pub = rospy.Publisher("/line_gps_y", Float64MultiArray, queue_size=1)

    gps_sub = rospy.Subscriber("/gps_crds", Float64MultiArray, GPS_CB, queue_size=1)
    imu_sub = rospy.Subscriber("/imu_data", Float64MultiArray, IMU_CB, queue_size=1)
    reset_sig_sub = rospy.Subscriber('/reset_signal', Bool, Reset_CB, queue_size=1)

    rospy.loginfo("[ ##### Line_Dectection_Online ##### ]")

    rate = rospy.Rate(30)
    LineDetection()
    # LD = LineDetection()
    # frame_tunning_ui = threading.Thread(target=frame_tunning, args=(LD,))
    # frame_tunning_ui.start()
    # color_tunning_ui = threading.Thread(target=color_tunning, args=(LD,))
    # color_tunning_ui.start()

    try:
        while(not rospy.is_shutdown()):
            line_x = Float64MultiArray()
            line_y = Float64MultiArray()

            if reset_signal:
                global_line_points = []

            if len(global_line_points) > 0:
                for crd in global_line_points:
                    line_x.data.append(crd[0])
                    line_y.data.append(crd[1])

            Line_GPS_X_pub.publish(line_x)
            Line_GPS_Y_pub.publish(line_y)
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("[ ##### Line_Dectection_Offline ##### ]")
        cv2.destroyAllWindows()
        # ui_thread.join()
        pass
