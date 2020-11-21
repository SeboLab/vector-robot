#!/usr/bin/env python3
from time import sleep
from threading import Event, Thread

from rospy import Publisher, Subscriber
from anki_vector_ros.msg import Pose, Proximity, RobotStatus, Touch
from std_msgs.msg import String
from anki_vector_ros.msg import Touch

class FistBump:
    def __init__(self):
        self.proximity_sub = Subscriber("/proximity", Proximity, self.callback)
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)

    def callback(self, proximity):
        if proximity.distance <= 50:
            self.counter += 1
        else:
            self.counter = 0
        if self.counter >= 10:
            self.anim_pub.publish("anim_fistbump_success_01")

