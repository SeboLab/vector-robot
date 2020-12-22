#!/usr/bin/env python3
from time import sleep
import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import Proximity, RobotStatus
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float32


class FistBump:
    def __init__(self):

        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.lift_pub = Publisher("/behavior/lift_height", Float32, queue_size=1)
        self.proximity_sub = Subscriber("/proximity", Proximity, self.proxim_callback)
        self.accel_sub = Subscriber("/accel", Vector3, self.accel_callback)

        self.bumped = False
        self.counter = 0
        self.previous_accel = None

        sleep(0.2)
        self.start_sequence()

    def start_sequence(self):
        self.lift_pub.publish(0.0)
        self.speech_pub.publish("Give me a fist bump")
        sleep(1.0)
        self.lift_pub.publish(0.6)

    def proxim_callback(self, proximity):
        # print(proximity.distance)
        if proximity.distance <= 60:
            self.counter += 1
        else:
            self.counter = 0

    def accel_callback(self, accel):
        print(accel.x)
        if (
            self.previous_accel is not None
            and not self.bumped
            and self.counter >= 1
            and abs(accel.x - self.previous_accel.x) > 800
        ):
            self.anim_pub.publish("anim_fistbump_success_01")
            self.speech_pub.publish("Wow! That was fun!")
            self.bumped = True
        self.previous_accel = accel


if __name__ == "__main__":
    rospy.init_node("fist_bump_test")
    rospy.wait_for_message("/status", RobotStatus)

    FistBump()
    rospy.spin()
