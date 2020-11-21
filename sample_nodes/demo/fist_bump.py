#!/usr/bin/env python3
from time import sleep
from threading import Event, Thread

from rospy import Publisher, Subscriber
from anki_vector_ros.msg import Pose, Proximity, RobotStatus, Touch
from std_msgs.msg import String
from anki_vector_ros.msg import Touch

class FistBump:
    def __init__(self):
        self.proximity_sub = Subscriber("/proximity", Proximity, callback)


    def callback(self, proximity):
        if proximity.distance <= 50:
            self.counter += 1
        else:
            counter = 0
        if counter >= 10:
            fistbump

