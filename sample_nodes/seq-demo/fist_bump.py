#!/usr/bin/env python3
from time import sleep
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import Proximity
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


class FistBump:
    def __init__(self):

        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.proximity_sub = Subscriber("/proximity", Proximity, self.proxim_callback)
        self.accel_sub = Subscriber("/accel", Vector3, self.accel_callback)
        self.bumped = False
        self.counter = 0
        self.start_sequence()
        self.previous_accel = None

    def start_sequence(self):
        self.speech_pub.publish("Give me a fist bump")

    def proxim_callback(self, proximity):
        print(proximity.distance)
        if proximity.distance <= 60:
            self.counter += 1
        else:
            self.counter = 0

    def accel_callback(self, accel):
        if (
            self.previous_accel is not None
            and not self.bumped
            and self.counter >= 1
            and abs(accel.x - self.previous_accel.x) > 500
        ):
            self.anim_pub.publish("anim_fistbump_success_01")
            self.speech_pub.publish("Wow! That was fun!")
            self.bumped = True
        self.previous_accel = accel


if __name__ == "__main__":
    import rospy

    rospy.init_node("fist_bump_test")
    FistBump()
    sleep(10)
