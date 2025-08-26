#!/usr/bin/env python3

"""

Bidirectional A* grid planning

author: Erwin Lejeune (@spida_rwin)

See Wikipedia article (https://en.wikipedia.org/wiki/Bidirectional_search)

"""

import math
import rospy

import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

show_animation = True

class BidirectionalAStarPlanner:
    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """
        self.state = True
        self.min_x, self.min_y = None, None
        self.max_x, self.max_y = None, None
        self.x_width, self.y_width, self.obstacle_map = None, None, None
        self.resolution = resolution
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        # self.state = True
        """
        Bidirectional A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set_A, closed_set_A = dict(), dict()
        open_set_B, closed_set_B = dict(), dict()
        open_set_A[self.calc_grid_index(start_node)] = start_node
        open_set_B[self.calc_grid_index(goal_node)] = goal_node

        current_A = start_node
        current_B = goal_node
        meet_point_A, meet_point_B = None, None

        while True:
            if len(open_set_A) == 0:
                # print("Open set A is empty..")
                break

            if len(open_set_B) == 0:
                # print("Open set B is empty..")
                break

            c_id_A = min(
                open_set_A,
                key=lambda o: self.find_total_cost(open_set_A, o, current_B))

            current_A = open_set_A[c_id_A]

            c_id_B = min(
                open_set_B,
                key=lambda o: self.find_total_cost(open_set_B, o, current_A))

            current_B = open_set_B[c_id_B]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current_A.x, self.min_x),
                         self.calc_grid_position(current_A.y, self.min_y),
                         "xc")
                plt.plot(self.calc_grid_position(current_B.x, self.min_x),
                         self.calc_grid_position(current_B.y, self.min_y),
                         "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set_A.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current_A.x == current_B.x and current_A.y == current_B.y:
                # print("Found goal")
                meet_point_A = current_A
                meet_point_B = current_B
                # self.state = False
                break

            # Remove the item from the open set
            del open_set_A[c_id_A]
            del open_set_B[c_id_B]

            # Add it to the closed set
            closed_set_A[c_id_A] = current_A
            closed_set_B[c_id_B] = current_B

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):

                c_nodes = [self.Node(current_A.x + self.motion[i][0],
                                     current_A.y + self.motion[i][1],
                                     current_A.cost + self.motion[i][2],
                                     c_id_A),
                           self.Node(current_B.x + self.motion[i][0],
                                     current_B.y + self.motion[i][1],
                                     current_B.cost + self.motion[i][2],
                                     c_id_B)]

                n_ids = [self.calc_grid_index(c_nodes[0]),
                         self.calc_grid_index(c_nodes[1])]

                # If the node is not safe, do nothing
                continue_ = self.check_nodes_and_sets(c_nodes, closed_set_A,
                                                      closed_set_B, n_ids)

                if not continue_[0]:
                    if n_ids[0] not in open_set_A:
                        # discovered a new node
                        open_set_A[n_ids[0]] = c_nodes[0]
                    else:
                        if open_set_A[n_ids[0]].cost > c_nodes[0].cost:
                            # This path is the best until now. record it
                            open_set_A[n_ids[0]] = c_nodes[0]

                if not continue_[1]:
                    if n_ids[1] not in open_set_B:
                        # discovered a new node
                        open_set_B[n_ids[1]] = c_nodes[1]
                    else:
                        if open_set_B[n_ids[1]].cost > c_nodes[1].cost:
                            # This path is the best until now. record it
                            open_set_B[n_ids[1]] = c_nodes[1]
        
        if meet_point_A is None or meet_point_B is None:
            rospy.logwarn("[ ##### No Path Found ##### ]")
            return [], []

        rx, ry = self.calc_final_bidirectional_path(
            meet_point_A, meet_point_B, closed_set_A, closed_set_B)

        return rx, ry

    # takes two sets and two meeting nodes and return the optimal path
    def calc_final_bidirectional_path(self, n1, n2, setA, setB):
        rx_A, ry_A = self.calc_final_path(n1, setA)
        rx_B, ry_B = self.calc_final_path(n2, setB)

        rx_A.reverse()
        ry_A.reverse()

        rx = rx_A + rx_B
        ry = ry_A + ry_B

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], \
                 [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def check_nodes_and_sets(self, c_nodes, closedSet_A, closedSet_B, n_ids):
        continue_ = [False, False]
        if not self.verify_node(c_nodes[0]) or n_ids[0] in closedSet_A:
            continue_[0] = True

        if not self.verify_node(c_nodes[1]) or n_ids[1] in closedSet_B:
            continue_[1] = True

        return continue_

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def find_total_cost(self, open_set, lambda_, n1):
        g_cost = open_set[lambda_].cost
        h_cost = self.calc_heuristic(n1, open_set[lambda_])
        f_cost = g_cost + h_cost
        return f_cost

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        global gps_x, gps_y
        # self.min_x = round(min(ox))
        # self.min_y = round(min(oy))
        # self.max_x = round(max(ox))
        # self.max_y = round(max(oy))
        self.min_x = round(gps_x - 20)
        self.min_y = round(gps_y - 20)
        self.max_x = round(gps_x + 20)
        self.max_y = round(gps_y + 20)
        # print("min_x:", self.min_x)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_x)
        # print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) * 2 / self.resolution)
        self.y_width = round((self.max_y - self.min_y) * 2 / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        pass

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

gps_x, gps_y = 0, 0
def GPS_CB(msg):
    global gps_x, gps_y
    gps_x = msg.data[0]
    gps_y = msg.data[1]

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

corrent_waypoint = []
waypoint_x, waypoint_y = 0, 0
def Corrent_Waypoint_CB(msg):
    global corrent_waypoint
    global waypoint_x, waypoint_y
    corrent_waypoint = msg.data
    waypoint_x = corrent_waypoint[0]
    waypoint_y = corrent_waypoint[1]

    # print(corrent_waypoint)

def Path_Maker_Pub():
    path = []
    path = list(zip(p_x, p_y))

    path_marker = Marker()
    path_array = MarkerArray()

    path_marker.header.frame_id = "world"
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "point"
    path_marker.id = 0
    path_marker.type = Marker.POINTS
    path_marker.action = Marker.ADD
    path_marker.scale.x = 0.15
    path_marker.scale.y = 0.15
    path_marker.scale.z = 0.15

    path_marker.pose.orientation.w = 1

    path_marker.color.r = 1.0
    path_marker.color.g = 0.0
    path_marker.color.b = 0.0
    path_marker.color.a = 1.0

    if len(path) > 0:
        for crds in path:
            points = Point(crds[0], crds[1], 0.1)
            path_marker.points.append(points)

    path_array.markers.append(path_marker)
    Path_Markers_pub.publish(path_array)

p_x, p_y = [], []
def main():
    global p_x, p_y
    global Line_GPS_X, Line_GPS_Y, Lidar_GPS_X, Lidar_GPS_Y
    # print(__file__ + " start!!")

    # start and goal position
    grid_size = 0.3  # [m]
    robot_radius = 0.3  # [m]

    try:
        while(not rospy.is_shutdown()):
            path_x = Float64MultiArray()
            path_y = Float64MultiArray()

            sx = gps_x  # [m]
            sy = gps_y  # [m]
            gx = waypoint_x  # [m]
            gy = waypoint_y  # [m]
            ox, oy = [], []

            for x in Line_GPS_X:
                ox.append(x)
            for y in Line_GPS_Y:
                oy.append(y)

            for x in Lidar_GPS_X:
                ox.append(x)
            for y in Lidar_GPS_Y:
                oy.append(y)

            if (len(ox) == (len(Line_GPS_X) + len(Lidar_GPS_X)) and len(oy) == len(Line_GPS_Y) + len(Lidar_GPS_Y)) and (not(sx == 0) and not(sy == 0)) and (not(gx == 0) and not(gy == 0)):
                if len(ox) > 0 and len(oy) > 0:
                    if show_animation:
                        plt.clf()
                        plt.plot(ox, oy, ".k")
                        plt.plot(sx, sy, "og")
                        plt.plot(gx, gy, "ob")
                        plt.grid(True)
                        plt.axis("equal")

                    bidir_a_star = BidirectionalAStarPlanner(ox, oy, grid_size, robot_radius)
                    p_x, p_y = bidir_a_star.planning(sx, sy, gx, gy)

                    path_x.data = p_x
                    path_y.data = p_y
                    Path_X_pub.publish(path_x)
                    Path_Y_pub.publish(path_y)

                    if show_animation:
                        plt.plot(p_x, p_y, "-r")
                        plt.pause(.0001)

            Path_Maker_Pub()
            # print(len(p_x))
            rospy.sleep(0.001)
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    rospy.init_node('path_finder', anonymous=False)
    Path_X_pub = rospy.Publisher("/path_x", Float64MultiArray, queue_size=1)
    Path_Y_pub = rospy.Publisher("/path_y", Float64MultiArray, queue_size=1)
    Path_Markers_pub = rospy.Publisher("/path_markers", MarkerArray, queue_size=1)

    gps_sub = rospy.Subscriber("/gps_crds", Float64MultiArray, GPS_CB, queue_size=1)

    Line_GPS_X_sub = rospy.Subscriber("/line_gps_x", Float64MultiArray, Line_GPS_X_CB, queue_size=1)
    Line_GPS_Y_sub = rospy.Subscriber("/line_gps_y", Float64MultiArray, Line_GPS_Y_CB, queue_size=1)
    Lidar_GPS_X_sub = rospy.Subscriber("/lidar_gps_x", Float64MultiArray, Lidar_GPS_X_CB, queue_size=1)
    Lidar_GPS_Y_sub = rospy.Subscriber("/lidar_gps_y", Float64MultiArray, Lidar_GPS_Y_CB, queue_size=1)
    Corrent_Waypoint_sub = rospy.Subscriber("/corrent_waypoint", Float64MultiArray, Corrent_Waypoint_CB, queue_size=1)

    rate = rospy.Rate(20)

    main()