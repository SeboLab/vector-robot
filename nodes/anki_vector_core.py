#!/usr/bin/env python3
"""A ROS node for interfacing with the Anki Vector's core sensors and motors.

This is a placeholder name for now; plan on dividing this up
based on functionality
"""

import rospy
import anki_vector
from geometry_msgs.msg import Vector3


class VectorNode:
    def __init__(self):
        # There are lots of parameters here for the robot
        # Perhaps we'll add these in a custom config file
        self.robot = anki_vector.Robot()
        self.robot.connect()

        self.accel_pub = rospy.Publisher("/accel", Vector3, queue_size=1)
        self.gyro_pub = rospy.Publisher("/gyro", Vector3, queue_size=1)

        while True:
            self.accel_pub.publish(self.robot.accel)
            self.accel_pub.publish(self.robot.gyro)

    def shutdown(self):
        print("Vector Robot shutting down...")
        self.robot.disconnect()


if __name__ == "__main__":
    rospy.init_node("vector")

    vector = VectorNode()
    rospy.spin()

    rospy.on_shutdown(vector.shutdown)
