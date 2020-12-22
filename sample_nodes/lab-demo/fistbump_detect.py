#!/usr/bin/env python3
"""Detect when fistbump has been done and react accordingly."""

from rospy import Subscriber
from anki_vector_ros.msg import Proximity
from geometry_msgs.msg import Vector3

from prompt_receiver import PromptReceiver, create_node


class FistBumpReceiver(PromptReceiver):
    def __init__(self):
        super().__init__()

        Subscriber("/proximity", Proximity, self.proxim_callback)
        Subscriber("/accel", Vector3, self.accel_callback)

        self.counter = 0
        self.previous_accel = None

    def proxim_callback(self, prox_msg):
        print("Proximity:", prox_msg.distance)
        if prox_msg.distance <= 60 and not prox_msg.is_lift_in_fov:
            self.counter += 1
        else:
            self.counter = 0

    def accel_callback(self, accel):
        if (
            self.previous_accel is not None
            and self.counter >= 1
            and abs(accel.x - self.previous_accel.x) > 600
            and not self.reacting
            and self.prompting
        ):
            self.fistbump_react()
        self.previous_accel = accel

    def fistbump_react(self):
        self.idle_pub.publish(False)
        self.prompt_pub.publish(False)
        self.reacting = True
        self.anim_pub.publish("anim_fistbump_success_01")
        self.speech_pub.publish("Wow! That was fun!")


if __name__ == "__main__":
    create_node("fist_bump_detector", FistBumpReceiver)
