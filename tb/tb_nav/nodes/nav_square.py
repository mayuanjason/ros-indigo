#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, sqrt, pow, pi
import PyKDL


class NavSquare:

    def __init__(self):
        # Give the node a name
        rospy.init_node('nav_square', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        self.robot_type = rospy.get_param("~robot_type", "turtlebot2")
        rospy.loginfo("robot type is " + self.robot_type)

        # How fast will we update the robot's movement
        rate = 20

        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)

        # Set the parameters for the target square
        goal_diatance = rospy.get_param("~goal_distance", 1.0)  # meters
        # degrees converted to radians
        goal_angle = radians(rospy.get_param("~goal_angle", 90))
        linear_speed = rospy.get_param("~linear_speed", 0.2)  # meters per second
        angular_speed = rospy.get_param("~angular_speed", 0.7)  # radians per second
        angular_tolerance = radians(rospy.get_param("~angular_tolerance", 2))  # degrees to radians

        # Publisher to control the robot's speed
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not uising Turtlebot2
        if self.robot_type == "turtlebot2":
            self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5)
        else:
            self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is base footprint for the TurtleBot
        self.base_frame = rospy.get_param("~base_frame", '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param("~odom_frame", '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo(
                "Cannot find transform between /dodm and /base_footprint")
            rospy.signal_shutdown("tf Exception")

        # Initialize the position variable as a Point type
        position = Point()

        # Cycle through the four sides of the square
        for i in range(4):
            # Initialize the movement command
            move_cmd = Twist()

            # Set the movement command to forward motion
            move_cmd.linear.x = linear_speed

            # Get the starting position values
            (position, rotation) = self.get_odom()

            x_start = position.x
            y_start = position.y

            # Keep track of the distance traveled
            distance = 0

            # Enter the loop to move along a side
            while distance < goal_diatance and not rospy.is_shutdown():
                # Publisher the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmd)
                r.sleep()

                # Get the current position
                (position, rotation) = self.get_odom()

                # Compute the Eculidean distance from the start
                distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

            # stop the robot before rotating
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

            # Set the movement command to a rotation
            move_cmd.angular.z = angular_speed

            # Track the last angle measured
            last_angle = rotation

            # Track how far we have turned
            turn_angle = 0

            # Begin the rotation
            while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmd)
                r.sleep()

                # Get the current rotation
                (position, rotation) = self.get_odom()

                # Compute the amount of rotation since the last loop
                delta_angle = self.normalize_angle(rotation - last_angle)

                # Add to the running total
                turn_angle += delta_angle
                last_angle = rotation

            # Stop the robot before the next leg
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

        # Stop the robot for good
        self.cmd_vel.publish(Twist())

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle

        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi

        return res

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        NavSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
