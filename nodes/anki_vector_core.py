#!/usr/bin/env python3
"""A ROS node for interfacing with the Anki Vector's core sensors and motors.

This is a placeholder name/format; plan on dividing this up
into different nodes/classes based on functionality
"""
from threading import Thread
import rospy

import anki_vector

from behavior import Behavior
from camera import Camera
from screen import Screen
from sensors import Sensors
from motors import Motors
from media import Media
from vision import Vision


class VectorNode:
    def __init__(self, publish_rate=10):
        self.rate = rospy.Rate(publish_rate)

        self.robot = anki_vector.Robot()
        self.robot.connect(timeout=20)

        self.behavior_control = Behavior(self.robot)
        self.screen_control = Screen(self.robot)
        self.motors_control = Motors(self.robot)
        self.media_control = Media(self.robot)
        self.vision_control = Vision(self.robot)

        self.async_robot = anki_vector.AsyncRobot()
        self.async_robot.connect()

        # Needs to be async due to continuous publishing
        # Thread(target=self.create_camera_thread).start()
        Thread(target=self.create_sensor_thread).start()

        # TODO handle NavMapComponents
        # World of the robot is best represented as Python objects; we can create
        # custom routines to represent LightCubes and other objects via IDs

    def create_camera_thread(self):
        Camera(self.robot)

    def create_sensor_thread(self):
        Sensors(self.robot, self.rate)

    def shutdown(self):
        print("Vector Robot shutting down...")
        self.robot.disconnect()


if __name__ == "__main__":
    rospy.init_node("vector_ros")

    vector = VectorNode()
    rospy.spin()

    rospy.on_shutdown(vector.shutdown)
