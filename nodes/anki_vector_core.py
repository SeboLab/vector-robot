#!/usr/bin/env python3
"""A ROS node for interfacing with the Anki Vector's core sensors and motors.

This is a placeholder name/format; plan on dividing this up
into different nodes/classes based on functionality
"""

import rospy
from std_msgs.msg import String, Int16, Bool
from geometry_msgs.msg import Vector3
from anki_vector_ros.msg import Dist

import anki_vector


class VectorNode:
    def __init__(self):
        # There are lots of parameters here for the robot
        # Perhaps we'll add these in a custom config file
        self.robot = anki_vector.Robot()
        self.robot.connect()

        self.accel_pub = rospy.Publisher("/accel", Vector3, queue_size=1)
        self.gyro_pub = rospy.Publisher("/gyro", Vector3, queue_size=1)

        # If we need any of the following to have responses (e.g. success/failure indicator),
        # we can transition them into services instead of topics

        # Animations
        # Could add list of animation names as topic, but not necessary for dev use
        self.anim_sub = rospy.Subscriber("/anim/play", String, self.play_anim)
        self.anim_trigger_sub = rospy.Subscriber(
            "/anim/play_trigger", String, self.play_anim_trigger
        )

        # Audio
        self.audio_sub = rospy.Subscriber("/audio/play", String, self.play_wav)
        self.audio_vol_sub = rospy.Subscriber("/audio/vol", Int16, self.set_vol)
        self.audio_vol = 100

        # Behavior
        self.drive_charger_sub = rospy.Subscriber(
            "/behavior/drive_charger", Bool, self.drive_charger
        )
        self.drive_straight_sub = rospy.Subscriber(
            "/behavior/drive_straight", Dist, self.drive_straight
        )
        # TODO: Finish writing subscribers for behavior actions

        while not rospy.is_shutdown():
            # Publish sensor data
            self.accel_pub.publish(self.robot.accel)
            self.accel_pub.publish(self.robot.gyro)

    def shutdown(self):
        print("Vector Robot shutting down...")
        self.robot.disconnect()

    def play_anim(self, str_data):
        self.robot.anim.play_animation(str_data)

    def play_anim_trigger(self, str_data):
        self.robot.anim.play_animation_trigger(str_data)

    def play_wav(self, str_data):
        self.robot.audio.stream_wav_file(str_data, self.audio_vol)

    def set_vol(self, vol):
        self.audio_vol = vol

    def drive_charger(self, bool_val):
        # True to drive on, false to drive off
        if bool_val:
            self.robot.behavior.drive_on_charger()
        else:
            self.robot.behavior.drive_off_charger()

    def drive_straight(self, msg):
        self.robot.behavior.drive_straight(msg.distance, msg.speed)


if __name__ == "__main__":
    rospy.init_node("vector")

    vector = VectorNode()
    rospy.spin()

    rospy.on_shutdown(vector.shutdown)
