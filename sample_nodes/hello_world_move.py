#!/usr/bin/env python3
from time import sleep

import rospy
from rospy import Publisher
from std_msgs.msg import Float32

"""
Sample program to make Vector move
"""


def main():
    print("Setting up publishers")
    move_pub = Publisher("/motors/wheels", Float32, queue_size=1)

    # Delay to ensure all topics are set up
    sleep(12.0)

    print("Executing commands")
    move_pub.publish(100.0)
    sleep(3.0)
    move_pub.publish(0.0)


if __name__ == "__main__":
    rospy.init_node("vector_hello_world")
    main()
