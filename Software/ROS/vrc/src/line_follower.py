#!/usr/bin/env python

"""Line follower

ROS node for a line follower robot running in simulation using V-REP.
"""

import rospy
import message_filters
from sensor_msgs.msg import Image, Range, Illuminance
from geometry_msgs.msg import Twist, Vector3
import numpy as np

__author__ = "Karl Kangur"
__email__ = "karl.kangur@gmail.com"


def process_linear_camera(image):
    """Process linear camera data."""
    # 3 pixels per camera
    pixels = np.fromstring(image.data, np.uint8)
    # Do something with the linear camera data


def process_ir_front(ir_left, ir_left_center, ir_right_center, ir_right):
    """Process front IR sensors."""
    # Do something with the front IR sensor data here
    ir_left_distance = ir_left.range
    ir_right_center_distance = ir_left_center.range
    ir_right_center_distance = ir_right_center.range
    ir_right_distance = ir_right.range


def process_ir_under(ir_left, ir_right):
    """Process IR sensors under the robot."""
    # Define the light threshold, 255 is light, 0 is dark
    light_threshold = 127
    # Define the speed constants
    speed_fast = 0.1
    speed_slow = 0.01
    speed_turn = 0.1
    speed_stop = 0.0

    # Define the robot behavior depending on sensor readings
    if (ir_left.illuminance < light_threshold and
            ir_right.illuminance < light_threshold):
        # If both sensors see the line be cautious
        v_linear = speed_slow
        v_angular = speed_stop
    elif ir_left.illuminance < light_threshold:
        # Line on the left, rotate counter-clockwise
        v_linear = speed_slow
        v_angular = speed_turn
    elif ir_right.illuminance < light_threshold:
        # Line on the right, rotate clockwise
        v_linear = speed_slow
        v_angular = -speed_turn
    else:
        # On the line, go fast
        v_linear = speed_fast
        v_angular = speed_stop

    # Build the Twist message sent to the robot
    msg = Twist()
    msg.linear = Vector3()
    msg.linear.x = v_linear
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular = Vector3()
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = v_angular

    # Never publish after the node has been shut down
    if not rospy.is_shutdown():
        # Actually publish the message
        pub.publish(msg)


def stop_robot():
    rospy.loginfo("Stopping robot")

    msg = Twist()
    msg.linear = Vector3()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular = Vector3()
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    # Actually publish the message
    pub.publish(msg)


def initialize():
    """Initialize the line follower node."""
    global pub

    # Provide a name for the node
    rospy.init_node("line_follower", anonymous=True)

    # Give some feedback
    rospy.loginfo("Line follower initialization")

    # Subscribe and synchronise to infra-red sensors in front of the robot
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
    ts_ir_front.registerCallback(process_ir_front)

    # Subscribe and synchronise to infra-red sensors under the robot
    ir_under_left = message_filters.Subscriber("ir_under_left", Illuminance)
    ir_under_right = message_filters.Subscriber("ir_under_right", Illuminance)
    # Wait for all topics to arrive before calling the callback
    ts_ir_under = message_filters.TimeSynchronizer([
        ir_under_left,
        ir_under_right], 1)
    ts_ir_under.registerCallback(process_ir_under)

    # Subscribe to the linear camera data
    rospy.Subscriber("linear_camera", Image, process_linear_camera)

    # Publish the linear and angular velocities
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    # When the node is stopped also stop the robot
    rospy.on_shutdown(stop_robot)

    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    initialize()
