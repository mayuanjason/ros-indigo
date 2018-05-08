#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import *
from std_msgs.msg import Float32
from tb_msgs.srv import *
import dynamic_reconfigure.server
from tb_utils.cfg import BatterySimulatorConfig
import thread


class BatterySimulator:

    def __init__(self):
        rospy.init_node("baterry_simulator")

        # The rate at which to publish the battrey level
        self.rate = rospy.get_param("~rate", 1)

        # Covert to a ROS rate
        r = rospy.Rate(self.rate)

        # The battery runtime in seconds
        self.battery_runtime = rospy.get_param("~battery_runtime", 30)  # seconds

        # The initial battery level - 100 is considered full charge
        self.initial_battery_level = rospy.get_param("~initial_battery_level", 100)

        # Error battery level for diagnostics
        self.error_battery_level = rospy.get_param("~error_battery_level", 20)

        # Warn battery level for diagnostics
        self.warn_battery_level = rospy.get_param("~warn_battery_level", 50)

        # Initialize the current level variable to the startup level
        self.current_battery_level = self.initial_battery_level

        # Initialize the new level variable to the startup level
        self.new_battery_level = self.initial_battery_level

        # The step sized used to decrease the battery level on each publishing loop
        self.battery_step = float(self.initial_battery_level) / (self.rate * self.battery_runtime)

        # Reserve a thread lock
        self.mutex = thread.allocate_lock()

        # Create the battery level publisher
        battery_level_pub = rospy.Publisher("battery_level", Float32, queue_size=5)

        # A service to manually set the battery level
        rospy.Service('~set_battery_level', SetBatteryLevel, self.SetBatteryLevelHandle)

        # Create a diagnostics publisher
        diag_pub = rospy.Publisher("diagnostics", DiagnosticArray, queue_size=5)

        # Create a dynamic_reconfigure server and set a callback function
        dyn_server = dynamic_reconfigure.server.Server(BatterySimulatorConfig, self.dynamic_reconfigure_callback)

        rospy.loginfo("Publishing simulated battery level with a runtime of " + str())

        # Start the publishing loop
        while not rospy.is_shutdown():
            # Initialize the diagnostics status
            status = DiagnosticStatus()
            status.name = "Battery Level"

            # Set the diagnostics status level based on the current battery level
            if self.current_battery_level < self.error_battery_level:
                status.message = "Low Battery"
                status.level = DiagnosticStatus.ERROR
            elif self.current_battery_level < self.warn_battery_level:
                status.message = "Medium Battery"
                status.level = DiagnosticStatus.WARN
            else:
                status.message = "Battery OK"
                status.level = DiagnosticStatus.OK

            # Add the raw battery level to the diagnostics message
            status.values.append(KeyValue("Battery Level", str(self.current_battery_level)))

            # Build the diagnostics array message
            msg = DiagnosticArray()
            msg.header.stamp = rospy.Time.now()
            msg.status.append(status)

            diag_pub.publish(msg)

            battery_level_pub.publish(self.current_battery_level)

            self.current_battery_level = max(0, self.current_battery_level - self.battery_step)

            r.sleep()

    def dynamic_reconfigure_callback(self, config, level):
        if self.battery_runtime != config['battery_runtime']:
            self.battery_runtime = config['battery_runtime']
            self.battery_step = 100.0 / (self.rate * self.battery_runtime)

        if self.new_battery_level != config['new_battery_level']:
            self.new_battery_level = config['new_battery_level']
            self.mutex.acquire()
            self.current_battery_level = self.new_battery_level
            self.mutex.release()

        return config

    def SetBatteryLevelHandle(self, req):
        self.mutex.acquire()
        self.current_battery_level = req.value
        self.mutex.release()
        return SetBatteryLevelResponse()


if __name__ == '__main__':
    BatterySimulator()
