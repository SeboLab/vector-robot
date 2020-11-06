#!/usr/bin/env python3
from time import sleep

import rospy
from rospy import Publisher
from std_msgs.msg import String, Float32

"""
Sample program to make Vector move and say "Hello"
"""


def main():
    speak_pub = Publisher("/audio/play", String, queue_size=1)
    speak_pub.publish("Hello!")

    move_pub = Publisher("/motors/wheels", Float32, queue_size=1)
    move_pub.publish(50.0)

    sleep(3.0)

    move_pub.publish(0.0)


if __name__ == "__main__":
    rospy.init_node("vector_hello_world")
    main()
