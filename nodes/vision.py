#!/usr/bin/env python3

from rospy import Subscriber
from std_msgs.msg import Bool


class Vision:
    def __init__(self, robot):
        self.robot = robot

        self.custom_obj_sub = Subscriber(
            "/vision/custom_object_detection", Bool, self.enable_object_detection
        )
        self.display_camera_sub = Subscriber(
            "/vision/display_camera_on_face", Bool, self.enable_face_cam_feed
        )
        self.face_detection_sub = Subscriber(
            "/vision/face_detection", Bool, self.enable_face_detection
        )

    def enable_object_detection(self, enable):
        self.robot.vision.enable_custom_object_detection(enable.data)

    def enable_face_cam_feed(self, enable):
        self.robot.vision.enable_display_camera_feed_on(enable.data)

    def enable_face_detection(self, enable):
        # Other parameters possible - could make this a custom message
        self.robot.vision.enable_face_detection(enable.data)
