#!/usr/bin/env python3
from time import sleep
from threading import Event, Thread
import numpy as np

from rospy import Publisher, Subscriber
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

"""
Example node of CV2 image connection
"""

# Brightness value (in HSV) threshold for sleeping
SLEEP_THRESH = 8.0


class TuckSleepRoutine:
    def __init__(self):
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.dark_img_count = 0

        Subscriber("/camera", Image, self.receive_camera)
        self.bridge = CvBridge()

        self.display_anim = Event()

        self.last_touch_status = False
        self.tucked = False
        sleep(0.08)

        self.start_anim()

    def start_anim(self):
        self.display_anim.clear()
        if not hasattr(self, "anim_thread"):
            self.speech_pub.publish("I'm getting sleepy...")
            self.anim_pub.publish("anim_gotosleep_getin_01")
            self.anim_thread = Thread(target=self.animate)
            self.anim_thread.start()

    def animate(self):
        # Add animation here as needed
        if not self.display_anim.isSet():
            pass
        self.display_anim.wait(1.0)

    def receive_camera(self, img_msg):
        # Receives image message and detects brightness
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        brightness = np.mean(cv_image)

        if brightness <= SLEEP_THRESH:
            self.dark_img_count += 1
        else:
            self.dark_img_count = 0

        if self.dark_img_count == 10:
            self.display_anim.set()
            self.speech_pub.publish("Thanks for tucking me in")

            self.anim_pub.publish("anim_gotosleep_sleeploop_01")
            sleep(5.0)
            self.anim_pub.publish("anim_neutral_eyes_01")
            self.tucked = True
