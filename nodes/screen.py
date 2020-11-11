#!/usr/bin/env python3
from rospy import Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import anki_vector
from anki_vector_ros.msg import Color


class Screen:
    def __init__(self, robot):
        self.robot = robot

        self.display_duration = 5.0
        self.color_sub = Subscriber("/screen/color", Color, self.set_color)
        self.image_sub = Subscriber("/screen/image", Image, self.set_image)
        self.display_dur_sub = Subscriber(
            "/screen/display_duration", Float32, self.set_display_duration
        )

    def set_color(self, color):
        color_obj = anki_vector.color.Color(rgb=[color.red, color.green, color.blue])
        self.robot.screen.set_screen_to_color(color_obj, self.display_duration)

    def set_image(self, image):
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

        screen_data = anki_vector.screen.convert_image_to_screen_data(cv_img)
        self.robot.screen.set_screen_with_image_data(screen_data, self.display_duration)

    def set_display_duration(self, duration):
        self.display_duration = duration.data
