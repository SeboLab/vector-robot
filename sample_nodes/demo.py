#!/usr/bin/env python3
from time import sleep

import rospy
from rospy import Publisher
from std_msgs.msg import String, Bool
from anki_vector_ros.msg import RobotStatus

"""
Sample program with a series of interactions with Vector
"""

pet_detected = False


def main():
    print("Setting up publishers")
    speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
    base_pub = Publisher("/behavior/drive_charger", Bool, queue_size=1)
    anim_pub = Publisher("/anim/play", String, queue_size=1)
    anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)

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

    while not pet_detected and not rospy.is_shutdown():
        sleep(6.0)
        anim_pub.publish("anim_explorer_scan_left_01")
        sleep(6.0)
        anim_pub.publish("anim_explorer_scan_right_01")


if __name__ == "__main__":
    rospy.init_node("vector_demo")
    rospy.wait_for_message("/status", RobotStatus)

    main()
