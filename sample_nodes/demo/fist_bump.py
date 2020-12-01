#!/usr/bin/env python3
from time import sleep
from threading import Event, Thread

from rospy import Publisher, Subscriber
from anki_vector_ros.msg import Pose, Proximity, RobotStatus, Touch
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from anki_vector_ros.msg import Touch

class FistBump:
    def __init__(self):

        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.proximity_sub = Subscriber("/proximity", Proximity, self.proxim_callback)
        self.accel_sub = Subscriber("/accel", Vector3, self.accel_callback)
        self.bumped = False
        self.start_sequence()


    def start_sequence(self):
        print("begin")
        self.speech_pub.publish("Give me a fist bump")

    def proxim_callback(self, proximity):
        if proximity.distance <= 60:
            self.counter += 1
        else:
            self.counter = 0
            

    def accel_callback(self, accel):
        print(accel.x)
        if self.counter >= 1 and abs(accel.x - self.previous_accel.x) > 100:
            print("fistbump")
            self.anim_pub.publish("anim_fistbump_success_01")
            self.speech_pub.publish("Hooray")
            self.bumped = True
        self.previous_accel = accel

