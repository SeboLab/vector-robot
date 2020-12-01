#!/usr/bin/env python3
from time import sleep

import rospy
from rospy import Publisher
from std_msgs.msg import String, Bool
from anki_vector_ros.msg import RobotStatus

from idle_anim import IdlePetAnimation
from sleep_routine import TuckSleepRoutine
from fist_bump import FistBump

"""
Sample program with a series of interactions with Vector
"""


class DemoNode:
    def __init__(self):

        print("Setting up publishers")
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.base_pub = Publisher("/behavior/drive_charger", Bool, queue_size=1)
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)

        sleep(0.08)
        self.init_drive()
        sleep(2.0)
        self.fist_bump_routine()
        self.pet_routine()
        self.tuck_sleep_routine()
        sleep(5.0)
        self.speech_pub.publish("Let's go to the next phase")
        self.next_phase()

    def init_drive(self):
        self.anim_trig_pub.publish("DriveStartHappy")
        self.base_pub.publish(False)
        sleep(8.0)

        self.speech_pub.publish("Hi, my name is Vector!")
        sleep(0.5)
        self.anim_pub.publish("anim_eyepose_happy")

    def pet_routine(self):
        anim = IdlePetAnimation()
        while not anim.petted:
            # Control rate of message being received here
            sleep(0.5)

        # Prevent further petting callbacks
        anim.touch_sub.unregister()

    def tuck_sleep_routine(self):
        state = TuckSleepRoutine()
        while not state.tucked:
            sleep(0.5)

    def fist_bump_routine(self):
        state = FistBump()
        while not state.bumped:
            sleep(.5)

    def next_phase(self):
        # Insert more here
        self.anim_pub.publish("anim_neutral_eyes_01")


if __name__ == "__main__":
    rospy.init_node("vector_demo")
    rospy.wait_for_message("/status", RobotStatus)

    DemoNode()
