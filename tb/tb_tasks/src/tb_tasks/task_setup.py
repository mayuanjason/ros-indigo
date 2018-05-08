#!/usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import pi
from collections import OrderedDict


def setup_task_environment(self):
    # How big is the square we want the robot to navigate?
    self.square_size = rospy.get_param("~square_size", 1.0)   # meters

    # Set the low battery threshold (between 0 and 100)
    self.low_battery_threshold = rospy.get_param("~low_battery_threshold", 50)

    # How many times should we execute the patrol loop
    self.n_patrols = rospy.get_param("~n_patrols", 2)

    # How long do we have to get to each waypoint?
    self.move_base_timeout = rospy.get_param("~move_base_timeout", 10)   # seconds

    # Initialize the patrol counter
    self.patrol_count = 0

    # Subscribe to the move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")

    # Wait for 60 seconds for the action server to become available
    self.move_base.wait_for_server(rospy.Duration(60))

    rospy.loginfo("Connected to move_base action server")
    
    # Create a list to hold the target quaternions (orientations)
    quaternions = list()

    # First define the corner orientations as Euler angles
    euler_angles = (pi / 2, pi, 3 * pi / 2, 0)

    # Then convert the angles to quaternions
    for angle in euler_angles:
        q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
        q = Quaternion(*q_angle)
        quaternions.append(q)

    # Create a list to hold the waypoint poses
    self.waypoints = list()

    # Append each of the four waypoints to the list. Each waypoint
    # is a pose consisting of a position and orientation in the map frame.
    self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
    self.waypoints.append(Pose(Point(self.square_size, 0.0, 0.0), quaternions[0]))
    self.waypoints.append(Pose(Point(self.square_size, self.square_size, 0.0), quaternions[1]))
    self.waypoints.append(Pose(Point(0.0, self.square_size, 0.0), quaternions[2]))

    # Create a mapping of room names to waypoint locations
    room_locations = (('hallway', self.waypoints[0]),
                      ('living_room', self.waypoints[1]),
                      ('kitchen', self.waypoints[2]),
                      ('bathroom', self.waypoints[3]))

    # Store 1the mapping as an ordered dictionary so we can visit the room in sequence
    self.room_locations = OrderedDict(room_locations)

    # Where is the docking station?
    self.docking_station_pose = Pose(Point(0.5, 0.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

    # Initialize the waypoint visualization markers for RViz
    init_waypoint_markers(self)

    # Set a visualization marker at each waypoint
    for waypoint in self.waypoints:
        p = Point()
        p = waypoint.position
        self.waypoint_markers.points.append(p)

    # Set a marker for the docking station
    init_docking_station_marker(self)

    # Publisher to manually control the robot (e.g. to stop it)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    rospy.loginfo("Starting Tasks")

    # Publish the waypoint markers
    self.marker_pub.publish(self.waypoint_markers)
    rospy.sleep(1)
    self.marker_pub.publish(self.waypoint_markers)

    # Publish the docking station marker
    self.docking_station_marker_pub.publish(self.docking_station_marker)
    rospy.sleep(1)

def init_waypoint_markers(self):
    # Set up our waypoint markers
    marker_scale = 0.2
    marker_lifetime = 0  # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

    # Define a marker publisher.
    self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)

    # Initialize the marker points list
    self.waypoint_markers = Marker()

    # Set the namespace and id for this marker. This serves to create a unique ID
    # Any marker sent with the same namespace and id will overwrite the old one
    self.waypoint_markers.ns = marker_ns
    self.waypoint_markers.id = marker_id

    self.waypoint_markers.type = Marker.CUBE_LIST

    # Set the marker action. Options are ADD/MODIFY(1), DELETE(2), DELETEALL(3)
    self.waypoint_markers.action = Marker.ADD

    # The lifetime field specifies how long this marker should stick around before being automatically deleted.
    # A value of ros::Duration() means never to auto-delete.
    self.waypoint_markers.lifetime = rospy.Duration(marker_lifetime)

    self.waypoint_markers.scale.x = marker_scale
    self.waypoint_markers.scale.y = marker_scale

    # Set the color, specified as r/g/b/a, with values in the range of [0, 1] -- Don't forget to set a or it will default to 0 and be invisible.
    # Note, an alpha (a) value of 0 means completely transparent (invisible), and 1 is completely opaque.
    self.waypoint_markers.color.r = marker_color['r']
    self.waypoint_markers.color.g = marker_color['g']
    self.waypoint_markers.color.b = marker_color['b']
    self.waypoint_markers.color.a = marker_color['a']

    # Set the frame ID and timestamp.
    self.waypoint_markers.header.frame_id = 'odom'
    self.waypoint_markers.header.stamp = rospy.Time.now()

    self.waypoint_markers.points = list()


def init_docking_station_marker(self):
    # Define a marker fir the charging station
    marker_scale = 0.3
    marker_lifetime = 0  # is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 0.7, 'g': 0.7, 'b': 0.0, 'a': 1.0}

    # Define a marker publisher.
    self.docking_station_marker_pub = rospy.Publisher('docking_station_marker', Marker, queue_size=5)

    # Initialize the marker points list
    self.docking_station_marker = Marker()

    # Set the namespace and id for this marker. This serves to create a unique ID
    # Any marker sent with the same namespace and id will overwrite the old one
    self.docking_station_marker.ns = marker_ns
    self.docking_station_marker.id = marker_id

    self.docking_station_marker.type = Marker.CYLINDER

    # Set the marker action. Options are ADD/MODIFY(1), DELETE(2), DELETEALL(3)
    self.docking_station_marker.action = Marker.ADD

    # The lifetime field specifies how long this marker should stick around before being automatically deleted.
    # A value of ros::Duration() means never to auto-delete.
    self.docking_station_marker.lifetime = rospy.Duration(marker_lifetime)

    self.docking_station_marker.scale.x = marker_scale
    self.docking_station_marker.scale.y = marker_scale
    self.docking_station_marker.scale.z = 0.02

    # Set the color, specified as r/g/b/a, with values in the range of [0, 1] -- Don't forget to set a or it will default to 0 and be invisible.
    # Note, an alpha (a) value of 0 means completely transparent (invisible), and 1 is completely opaque.
    self.docking_station_marker.color.r = marker_color['r']
    self.docking_station_marker.color.g = marker_color['g']
    self.docking_station_marker.color.b = marker_color['b']
    self.docking_station_marker.color.a = marker_color['a']

    # Set the frame ID and timestamp.
    self.docking_station_marker.header.frame_id = 'odom'
    self.docking_station_marker.header.stamp = rospy.Time.now()

    self.docking_station_marker.pose = self.docking_station_pose
