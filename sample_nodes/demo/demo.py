#!/usr/bin/env python3
from time import sleep

import rospy
from rospy import Publisher, Subscriber
from std_msgs.msg import String, Bool
from anki_vector_ros.msg import RobotStatus, Touch
from idle_anim import IdlePetAnimation

"""
Sample program with a series of interactions with Vector
"""


def on_touch(touch_msg):
    pass


def main():
    print("Setting up publishers")
    speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
    base_pub = Publisher("/behavior/drive_charger", Bool, queue_size=1)
    anim_pub = Publisher("/anim/play", String, queue_size=1)
    anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)

    touch_sub = Subscriber("/touch", Touch, on_touch)

    # Need small delay to setup publishers
    sleep(0.08)
    anim_trig_pub.publish("DriveStartHappy")
    base_pub.publish(False)
    sleep(8.0)

    speech_pub.publish("Hi, my name is Vector!")
    sleep(0.5)
    anim_pub.publish("anim_eyepose_happy")
    sleep(2.0)
    speech_pub.publish("Do you want to pet me?")

    anim = IdlePetAnimation()
    while not anim.petted:
        sleep(0.5)

    sleep(1.0)
    speech_pub.publish("Let's go to the next phase")


if __name__ == "__main__":
    rospy.init_node("vector_demo")
    rospy.wait_for_message("/status", RobotStatus)

    main()
    rospy.spin()
