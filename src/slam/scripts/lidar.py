#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Lidar:
    def __init__(self,topic_lidar,topic_position):
        self.name = 'diff_drive'
        self.sub_lidar = rospy.Subscriber(topic_lidar,LaserScan,self.callback_lidar)
        self.sub_position = rospy.Subscriber(topic_position,Odometry,self.callback_position)
        self.distances = []
        self.range_min = 0
        self.range_max = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.position = [self.x,self.y,self.theta]

    def callback_lidar(self,lidar):
        self.range_min = lidar.range_min
        self.range_max = lidar.range_max
        ranges = lidar.ranges
        self.distances = ['' if x =='inf' or x < self.range_min or x > self.range_max else x for x in ranges]
    
    def callback_position(self,pos):
        self.x,self.y = pos.pose.pose.position.x,pos.pose.pose.position.y
        self.theta = self.quat_to_z_euler(pos.pose.pose.orientation.x,pos.pose.pose.orientation.y,pos.pose.pose.orientation.z,pos.pose.pose.orientation.w,False)
        self.position = [self.x,self.y,self.theta]

    def quat_to_z_euler(self,x,y,z,w,degrees):
        t1 = 2.0 * (w * z + x * y)
        t2 = 1.0 -2.0 * (y * y + z * z)
        if degrees:
            z = np.degrees(np.arctan2(t1,t2))
            if z >= 0:
                return z
            else:
                return 180.0 - z
        else:
            return np.arctan2(t1,t2)
