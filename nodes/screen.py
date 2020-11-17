#!/usr/bin/env python3
from rospy import Subscriber
from PIL import Image
from std_msgs.msg import Float32, String

import anki_vector
from anki_vector_ros.msg import Color


class Screen:
    def __init__(self, robot):
        self.robot = robot

        self.display_duration = 5.0
        self.color_sub = Subscriber("/screen/color", Color, self.set_color)
        self.image_sub = Subscriber("/screen/image", String, self.set_image)
        self.display_dur_sub = Subscriber(
            "/screen/display_duration", Float32, self.set_display_duration
        )

    def set_color(self, color):
        color_obj = anki_vector.color.Color(rgb=[color.red, color.green, color.blue])
        self.robot.screen.set_screen_to_color(color_obj, self.display_duration)

    def set_image(self, image):
        # Max width: 184, max height: 96
        img_file = Image.open(image.data)
        w, h = img_file.size
        resize_ratio = min(184 / w, 96 / h)
        new_size = (int(resize_ratio * w), int(resize_ratio * h))
        img_file = img_file.resize(new_size, Image.ANTIALIAS)

        canvas = Image.new("RGB", (184, 96), (0, 0, 0))

        w, h = new_size
        tl_corner = ((184 - w) // 2, (96 - h) // 2)

        canvas.paste(img_file, tl_corner)

        screen_data = anki_vector.screen.convert_image_to_screen_data(canvas)
        self.robot.screen.set_screen_with_image_data(screen_data, self.display_duration)

    def set_display_duration(self, duration):
        self.display_duration = duration.data
