#!/usr/bin/env python
"""Exploration node

ROS node for an exploration robot running in simulation using V-REP.
"""

import rospy
import message_filters
import random
import time
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, Vector3

__author__ = "Karl Kangur"
__email__ = "karl.kangur@gmail.com"


def process_ir_front(ir_left, ir_left_center, ir_right_center, ir_right):
    """Do obstacle avoidance."""
    # Define the distance to obstacle constant
    distance_obstacle = 0.15
    # Define the speed constants
    speed_fast = 0.1
    speed_slow = 0.01
    speed_turn = 0.5
    speed_stop = 0.0
    # Define the time constants
    time_turn = 2.0

    # When the obstacle is detected turn on itself with a random speed
    if (ir_left_center.range < distance_obstacle or
            ir_left.range < distance_obstacle):
        # Inform human
        rospy.loginfo("Obstacle detected on the left")
        # Stop
        v_linear = speed_stop
        # Turn clockwise
        v_angular = -speed_turn
        # Set the speed
        set_speed(v_linear, v_angular)
        # Wait for a random amount of time
        time.sleep(time_turn * random.random())
    elif (ir_right_center.range < distance_obstacle or
            ir_right.range < distance_obstacle):
        rospy.loginfo("Obstacle detected on the right")
        v_linear = speed_stop
        v_angular = speed_turn
        set_speed(v_linear, v_angular)
        time.sleep(time_turn * random.random())
    else:
        # Go straight forward
        v_linear = speed_fast
        v_angular = speed_stop
        set_speed(v_linear, v_angular)


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
    rospy.init_node("explore", anonymous=True)

    # Give some feedback in the terminal
    rospy.loginfo("Exploration node initialization")

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

    # Publish the linear and angular velocities so the robot can move
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    # Register the callback for when the node is stopped
    rospy.on_shutdown(stop_robot)

    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    initialize()
