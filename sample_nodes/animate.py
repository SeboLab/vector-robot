#!/usr/bin/env python3
from time import sleep

import rospy
from rospy import Publisher
from std_msgs.msg import String, Float32, Bool
from anki_vector_ros.msg import RobotStatus, Color

"""
Sample program to make Vector animate off the base
"""


def main():
    print("Setting up publishers")
    speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
    lift_pub = Publisher("/motors/lift", Float32, queue_size=5)
    base_pub = Publisher("/behavior/drive_charger", Bool, queue_size=1)
    spin_pub = Publisher("/behavior/turn_in_place", Float32, queue_size=1)
    eye_color_pub = Publisher("/behavior/eye_color", Color, queue_size=1)
    # Need small delay to setup publishers
    sleep(1.0)
    lift_pub.publish(-2.0)
    sleep(0.08)
    for _ in range(10):
        lift_pub.publish(2.0)
        sleep(0.08)
        lift_pub.publish(0.0)
        sleep(0.08)
        lift_pub.publish(-2.0)
        sleep(0.08)
        lift_pub.publish(0.0)
        sleep(0.08)

    base_pub.publish(False)
    sleep(5.0)

    speech_pub.publish("Hi, my name is Vector!")
    sleep(1.8)

    eye_color_pub.publish(Color(255, 0, 0))
    sleep(1.5)
    spin_pub.publish(3.14)
    sleep(2.0)
    eye_color_pub.publish(Color(128, 128, 0))


if __name__ == "__main__":
    rospy.init_node("vector_animate")
    rospy.wait_for_message("/status", RobotStatus)

    main()
