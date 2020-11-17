#!/usr/bin/env python3
from time import sleep
import sys

import rospy
from rospy import Publisher
from std_msgs.msg import Float32
from anki_vector_ros.msg import RobotStatus

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

"""
Sample program to display an image on Vector's screen

Usage:
python3 display_image.py /path/to/img.png
"""


def display_image(file):
    print("Setting up publishers")
    img_pub = Publisher("/screen/image", Image, queue_size=1)
    dur_pub = Publisher("/screen/display_duration", Float32, queue_size=1)

    # Need small delay to setup publishers
    sleep(0.5)

    dur_pub.publish(float(input("Enter display duration [s]: ")))

    img = cv2.imread(file, cv2.IMREAD_COLOR)
    bridge = CvBridge()

    img_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("vector_image")
    rospy.wait_for_message("/status", RobotStatus)

    display_image(sys.argv[1])
