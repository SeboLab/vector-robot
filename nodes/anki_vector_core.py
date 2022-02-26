#!/usr/bin/env python3
"""A ROS node for interfacing with the Anki Vector's core sensors and motors."""
import sys
import argparse
from threading import Thread
import concurrent.futures
import rospy

import anki_vector

from behavior import Behavior
from camera import Camera
from screen import Screen
from sensors import Sensors
from motors import Motors
from media import Media
from vision import Vision
from events import EventHandler
from cube import LightCube


class VectorNode:
    def __init__(self, publish_rate=10, camera=False, serial=None):
        self.rate = rospy.Rate(publish_rate)

        self.robot = anki_vector.Robot(enable_face_detection=camera, serial=serial)

        try:
            self.robot.connect(timeout=20)
        except anki_vector.exceptions.VectorNotFoundException:
            print("ERROR: Unable to establish a connection to Vector.")
            print(
                "Make sure you're on the same network, and Vector is connected to the internet."
            )
            sys.exit(1)

        self.behavior_control = Behavior(self.robot)
        self.screen_control = Screen(self.robot)
        self.motors_control = Motors(self.robot)
        self.media_control = Media(self.robot)
        self.vision_control = Vision(self.robot)
        self.event_handler = EventHandler(self.robot)

        # Create async threads due to continuous publishing
        Thread(target=Sensors, args=[self.robot, self.rate]).start()
        Thread(target=LightCube, args=[self.robot, self.rate]).start()
        if camera:
            Thread(target=Camera, args=[self.robot]).start()


    def shutdown(self):
        print("Vector Robot shutting down...")
        self.robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", action="store_true")
    parser.add_argument("--serial", default=None, nargs="?", type=str)
    args, _ = parser.parse_known_args()

    if args.serial == "NULL":
        args.serial = None

    rospy.init_node("vector_ros")

    vector = VectorNode(camera=args.camera, serial=args.serial)
    rospy.spin()

    try:
        rospy.on_shutdown(vector.shutdown)
    except (concurrent.futures.CancelledError, RuntimeError):
        sys.exit(0)
