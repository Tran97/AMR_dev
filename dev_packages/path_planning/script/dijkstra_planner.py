#!/usr/bin/env python

import rospy
import heapq
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import PointStamped
import numpy as np
import math
from threading import Lock

class DijkstraPathPlanner:
    def __init__(self):
        rospy.init_node('dijkstra_path_planner')
        self.mutex = Lock()

        self.map_sub = rospy.Subscriber('/occ_map', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        self.start_sub = rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.start_callback, queue_size=1)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        self.map = None
        self.resolution = None
        self.origin = None
        self.goal = None
        self.start = None

    def map_callback(self, msg):
        print("Planner got map")
        self.map = msg

    def goal_callback(self, goal_msg):
        self.goal = goal_msg.pose.position
        #with self.mutex:
        self.mutex.acquire(blocking=True)
        self.compute_and_publish_path()
        self.mutex.release()

    def start_callback(self, start_msg):
        self.start = start_msg.pose.pose.position
        #with self.mutex:
        self.mutex.acquire(blocking=True)
        self.compute_and_publish_path()
        self.mutex.release()

    def f_value(self, s):
        """
        f = g. (g: Cost to come)
        :param s: current state
        :return: f
        """

        return self.g[s]

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def dijkstra(self):
        if self.map is None or self.goal is None or self.start is None:
            rospy.logwarn("Map, goal, or start not yet received.")
            return None
        print("Planner computing...", (self.start.x,self.start.y))

        start_x = int(self.start.x)
        start_y = int(self.start.y)
        goal_x =  int(self.goal.x) 
        goal_y =  int(self.goal.y) 

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

        self.s_start = (start_x, start_y)
        self.s_goal = (goal_x, goal_y)
        
        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbors(s[0], s[1]):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        path = self.extract_path(self.PARENT)
        path.reverse()
        return path

    def compute_and_publish_path(self):
        path = self.dijkstra()
        if path:
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"
            for point in path:
                pose = PoseStamped()
                pose.pose.position.x = point[0] 
                pose.pose.position.y = point[1]
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)

    def cartesian_to_grid_indices(self, x, y, map_info):
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        map_width = map_info.width

        # Convert Cartesian coordinates to grid coordinates
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        # Convert grid coordinates to linear index
        index = grid_y * map_width + grid_x

        return int(index)

    def get_neighbors(self, x, y):
        neighbors = []
        x = x
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_x, new_y = x + dx, y + dy
                index = self.cartesian_to_grid_indices(new_x,new_y,self.map.info)
                if 0 <= index < len(self.map.data) and self.map.data[index] < 1:
                    neighbors.append((new_x, new_y))
        return neighbors

    def cost(self, current, next):
        return np.linalg.norm(np.array(next) - np.array(current))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    planner = DijkstraPathPlanner()
    planner.run()
