#!/usr/bin/env python3
"""Example node of idle animation detection and interruption."""

from time import sleep
from threading import Event, Thread

from rospy import Publisher, Subscriber
from std_msgs.msg import String
from anki_vector_ros.msg import Touch


class IdlePetAnimation:
    def __init__(self):
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.touch_sub = Subscriber("/touch", Touch, self.on_touch)

        self.display_anim = Event()

        self.last_touch_status = False
        self.petted = False
        sleep(0.08)

        self.start_anim()

    def start_anim(self):
        self.display_anim.clear()
        if not hasattr(self, "anim_thread"):
            self.speech_pub.publish("Do you want to pet my back?")
            self.anim_thread = Thread(target=self.animate)
            self.anim_thread.start()

    def animate(self):
        while not self.display_anim.isSet():
            self.anim_pub.publish("anim_explorer_scan_left_01")
            self.display_anim.wait(7.0)
            if not self.display_anim.isSet():
                self.anim_pub.publish("anim_explorer_scan_right_01")
            self.display_anim.wait(7.0)

    def on_touch(self, touch_msg):
        # This represents any trigger for shutting down animation
        if not touch_msg.is_being_touched:
            self.last_touch_status = False
            return
        self.display_anim.set()

        if not self.last_touch_status:
            # If continuous petting, don't say phrase
            self.anim_pub.publish("anim_petting_lvl1_01")
            self.speech_pub.publish("Ooh, that feels good!")
            self.petted = True
        self.last_touch_status = True
