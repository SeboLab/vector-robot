#!/usr/bin/env python3
from time import sleep

import rospy
from rospy import Publisher
from std_msgs.msg import String
from anki_vector_ros.msg import RobotStatus

"""
Sample program to make Vector move
"""


def main():
    print("Setting up publishers")
    speech_pub = Publisher("/behavior/say_text", String, queue_size=1)

    # Need small delay to setup publishers
    sleep(0.5)

    while True:
        speech_pub.publish(input("Enter phrase: "))


if __name__ == "__main__":
    rospy.init_node("vector_speech")
    rospy.wait_for_message("/status", RobotStatus)

    main()
