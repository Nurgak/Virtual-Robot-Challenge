#!/usr/bin/env python
"""Sample node

Sample ROS node for the robot running in simulation using V-REP.
"""

import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Image, Range, Illuminance
from geometry_msgs.msg import Twist, Vector3

__author__ = "Karl Kangur"
__email__ = "karl.kangur@gmail.com"


def process_linear_camera(image):
    """Process linear camera data."""
    # Do something with the linear camera data, 3 pixels per camera
    pixels = np.fromstring(image.data, np.uint8)


def process_ir_front(ir_left, ir_left_center, ir_right_center, ir_right):
    """Process front IR sensors."""
    # Do something with the front IR sensors, values are in meters
    ir_left_distance = ir_left.range
    ir_right_center_distance = ir_left_center.range
    ir_right_center_distance = ir_right_center.range
    ir_right_distance = ir_right.range


def process_ir_under(ir_left, ir_right):
    """Process IR sensors under the robot."""
    # Do something with the sensors under the robot. 255 is light, 0 is dark
    ir_left_intensity = ir_left.illuminance
    ir_right_intensity = ir_right.illuminance


def set_speed(linear, angular):
    # Set robot speed with a Twist topic
    msg = Twist()
    msg.linear = Vector3()
    msg.linear.x = linear
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular = Vector3()
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = angular

    # Actually publish the topic
    pub.publish(msg)


def stop_robot():
    rospy.loginfo("Stopping robot")
    set_speed(0, 0)


def initialize():
    """Initialize the sample node."""
    global pub

    # Provide a name for the node
    rospy.init_node("sample", anonymous=True)

    # Give some feedback in the terminal
    rospy.loginfo("Sample node initialization")

    # Subscribe to and synchronise the infra-red sensors in front of the robot
    ir_front_left = message_filters.Subscriber("ir_front_left", Range)
    ir_front_right = message_filters.Subscriber("ir_front_right", Range)
    ir_front_left_center = message_filters.Subscriber(
        "ir_front_left_center", Range)
    ir_front_right_center = message_filters.Subscriber(
        "ir_front_right_center", Range)
    # Wait for all topics to arrive before calling the callback
    ts_ir_front = message_filters.TimeSynchronizer([
        ir_front_left,
        ir_front_left_center,
        ir_front_right_center,
        ir_front_right], 1)
    # Register the callback to be called when all sensor readings are ready
    ts_ir_front.registerCallback(process_ir_front)

    # Subscribe to and synchronise the infra-red sensors under the robot
    ir_under_left = message_filters.Subscriber("ir_under_left", Illuminance)
    ir_under_right = message_filters.Subscriber("ir_under_right", Illuminance)
    # Wait for all topics to arrive before calling the callback
    ts_ir_under = message_filters.TimeSynchronizer([
        ir_under_left,
        ir_under_right], 1)
    # Register the callback to be called when all sensor readings are ready
    ts_ir_under.registerCallback(process_ir_under)

    # Subscribe to the linear camera data
    rospy.Subscriber("linear_camera", Image, process_linear_camera)

    # Publish the linear and angular velocities so the robot can move
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    # Register the callback for when the node is stopped
    rospy.on_shutdown(stop_robot)

    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    initialize()
