#!/usr/bin/env python3
"""Detect when fistbump has been done and react accordingly."""

import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import Proximity, RobotStatus, Response
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Bool


class FistBumpReceiver:
    def __init__(self):

        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.idle_pub = Publisher("/labdemo/idle", Bool, queue_size=1)

        Subscriber("/proximity", Proximity, self.proxim_callback)
        Subscriber("/accel", Vector3, self.accel_callback)
        Subscriber("/labdemo/idle", Bool, self.idle_callback)
        Subscriber("/behavior/response", Response, self.process_behavior_response)

        self.counter = 0
        self.previous_accel = None
        self.idling = True
        self.reacting = False

    def proxim_callback(self, proximity):
        print("Proximity:", proximity.distance)
        if proximity.distance <= 60:
            self.counter += 1
        else:
            self.counter = 0

    def accel_callback(self, accel):
        if (
            self.previous_accel is not None
            and self.counter >= 1
            and abs(accel.x - self.previous_accel.x) > 800
            and not self.reacting
        ):
            self.fistbump_react()
        self.previous_accel = accel

    def idle_callback(self, msg):
        self.idling = msg.data

    def fistbump_react(self):
        self.idle_pub.publish(False)
        self.reacting = True
        self.anim_pub.publish("anim_fistbump_success_01")
        self.speech_pub.publish("Wow! That was fun!")

    def process_behavior_response(self, resp_msg):
        if (
            resp_msg.type == "SayTextResponse"
            and self.reacting
            and resp_msg.result == 0
        ):
            # Animation sequence has terminated
            self.idle_pub.publish(True)
            self.reacting = False


if __name__ == "__main__":
    rospy.init_node("fist_bump_detector")
    rospy.wait_for_message("/status", RobotStatus)

    FistBumpReceiver()
    rospy.spin()
