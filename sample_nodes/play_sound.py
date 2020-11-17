#!/usr/bin/env python3
from time import sleep
import sys

import rospy
from rospy import Publisher
from std_msgs.msg import String, Int16
from anki_vector_ros.msg import RobotStatus

"""
Sample program to make Vector play an audio file

Usage:
python3 play_sound.py /absolute/dir/to/sound.wav
"""


def play_file(file):
    print("Setting up publishers")
    play_pub = Publisher("/audio/play", String, queue_size=1)
    vol_pub = Publisher("/audio/vol", Int16, queue_size=1)
    

    # Need small delay to setup publishers
    sleep(0.5)

    
    vol_pub.publish(int(input("Enter volume (0-100): ")))
    play_pub.publish(file)

if __name__ == "__main__":
    rospy.init_node("vector_sound")
    rospy.wait_for_message("/status", RobotStatus)

    play_file(sys.argv[1])
