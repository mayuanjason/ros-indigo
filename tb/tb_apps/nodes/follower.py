#!/usr/bin/env python

import rospy
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs import Twist
from math import copysign


class Follower():

    def __init__(self):
        rospy.init_node("follower")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # The dimensions (in meters) of the box in which we will search
        # for the person (blob). These are given in camera coordinates
        # Where x is left/right, y is up/down and z is depth (forward/backward)
        self.min_x = rospy.get_param("~min_x", -0.2)
        self.max_x = rospy.get_param("~max_x", 0.2)
        self.min_y = rospy.get_param("~min_y", -0.3)
        self.max_y = rospy.get_param("~max_x", -0.5)
        self.max_z = rospy.get_param("~max_z", 1.2)

        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 0.6)

        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)

        # How far away from being centered (x displacement) on the person before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)

        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight left/right displacement of the person when making a movement
        self.x_scale = rospy.get_param("~x_scale", 2.5)

        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)

        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)

        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)

        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)

        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)

        # Initialize the movement command
        self.move_cmd = Twist()

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the point cloud
        self.depth_subscriber = rospy.Subscriber('point_cloud', PointCloud2, self.set_cmd_vel, queue_size=1)

        rospy.loginfo("Subscribing to point cloud...")

        # Wait for the point_cloud topic to become available
        rospy.wait_for_message('point_cloud', PointCloud2)

        rospy.loginfo("Ready to follow!")

    def set_cmd_vel(self, msg):
        pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()

        # Send an empty Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ = '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")