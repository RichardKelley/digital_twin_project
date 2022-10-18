#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import py_trees
import py_trees_ros

from map_manager.msg import Atlas, Landmark
from map_manager.srv import *

import torch

import smtplib, ssl

port = 465
password = input("Please enter your password and press enter: ")
context = ssl.create_default_context()

blackboard = py_trees.blackboard.Blackboard()

class CheckLandmarkProximity(py_trees.Behavior):
    def __init__(self, name):
        super().__init__(name)

    def setup(self, timeout):
        pass

    def initialize(self):
        pass

    def update(self):
        pass

    def terminate(self):
        pass


def get_landmark_cloud(cloud_id):
    pass

def odom_cb(data):
    loc_x = data.pose.pose.position.x
    loc_y = data.pose.pose.position.y
    blackboard.loc = torch.tensor([loc_x, loc_y])

def main():
    rospy.init_node('ai_monitor', anonymous=True)

    rospy.Subscriber("localization", Odometry, odom_cb)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
