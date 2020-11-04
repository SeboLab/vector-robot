#!/usr/bin/env python3
"""A ROS node for interfacing with the Anki Vector's core sensors and motors.

This is a placeholder name/format; plan on dividing this up
into different nodes/classes based on functionality
"""
from threading import Thread
import rospy
from rospy import Subscriber, Publisher
from std_msgs.msg import String, Int16, Float32, Bool
from geometry_msgs.msg import Vector3

import anki_vector

from behavior import Behavior
from camera import Camera
from anki_vector_ros.msg import Pose


def create_camera_thread(robot):
    Camera(robot)


class VectorNode:
    def __init__(self):
        # There are lots of parameters here for the robot
        # Perhaps we'll add these in a custom config file
        self.robot = anki_vector.Robot()
        self.robot.connect()

        self.behavior_control = Behavior(self.robot)

        self.async_robot = anki_vector.AsyncRobot(enable_camera_feed=True)
        self.async_robot.connect()

        # Needs to be async due to continuous publishing
        Thread(target=create_camera_thread, args=(self.async_robot)).start()

        # Animations
        # Could add list of animation names as topic, but not necessary for dev use
        self.anim_sub = Subscriber("/anim/play", String, self.play_anim)
        self.anim_trigger_sub = Subscriber(
            "/anim/play_trigger", String, self.play_anim_trigger
        )

        # Audio
        self.audio_sub = Subscriber("/audio/play", String, self.play_wav)
        self.audio_vol_sub = Subscriber("/audio/vol", Int16, self.set_vol)
        self.audio_vol = 100

        # Motors
        self.head_motor_sub = Subscriber("/motors/head", Float32, self.set_head_motor)
        self.lift_motor_sub = Subscriber("/motors/lift", Float32, self.set_lift_motor)
        self.wheel_motors_sub = Subscriber(
            "/motors/wheels", Float32, self.set_wheel_motors
        )
        self.stop_motors_sub = Subscriber("/motors/stop", Bool, self.stop_motors)

        # Info publishing
        self.accel_pub = Publisher("/accel", Vector3, queue_size=1)
        self.gyro_pub = Publisher("/gyro", Vector3, queue_size=1)
        self.carry_object_pub = Publisher("/carry_object", Int16)
        self.angle_pub = Publisher("/head_angle", Float32, queue_size=1)
        self.tracking_pub = Publisher("/head_tracking_object", Int16, queue_size=1)
        self.left_wheel_pub = Publisher("/wheel_speed/left", Float32, queue_size=1)
        self.right_wheel_pub = Publisher("/wheel_speed/right", Float32, queue_size=1)
        self.lift_height_pub = Publisher("/lift_height", Float32, queue_size=1)
        self.localized_pub = Publisher("/localized_object", Int16)
        self.pose_pub = Publisher("/pose", Pose)
        self.pose_angle_pub = Publisher("/pose/angle", Float32)
        self.pose_pitch_pub = Publisher("/pose/pitch", Float32)

        # TODO handle NavMapComponents
        # TODO continue with ProximityComponent in
        # https://developer.anki.com/vector/docs/generated/anki_vector.robot.html#module-anki_vector.robot
        # TODO create helper function to translation between anki_vector data object
        # and a ROS message

        while not rospy.is_shutdown():
            # Publish sensor/motor data
            self.accel_pub.publish(self.robot.accel)
            self.gyro_pub.publish(self.robot.gyro)
            self.angle_pub.publish(self.robot.head_angle_rad)
            self.tracking_pub.publish(self.robot.head_tracking_object_id)
            self.left_wheel_pub.publish(self.robot.left_wheel_speed_mmps)
            self.right_wheel_pub.publish(self.robot.right_wheel_speed_mmps)
            self.lift_height_pub.publish(self.robot.lift_height_mm)
            self.localized_pub.publish(self.robot.localized_to_object_id)
            self.carry_object_pub.publish(self.robot.carrying_object_id)

            pose_msg = Pose()
            for attr in dir(self.robot.pose):
                setattr(pose_msg, attr, getattr(self.robot.pose, attr))

            self.pose_pub.publish(pose_msg)
            self.pose_angle_pub.publish(self.robot.pose_angle_rad)
            self.pose_pitch_pub.publish(self.robot.pose_pitch_rad)

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

    def set_head_motor(self, speed):
        self.robot.motors.set_head_motor(speed)

    def set_lift_motor(self, speed):
        self.robot.motors.set_lift_motor(speed)

    def set_wheel_motors(self, speed):
        self.robot.motors.set_wheel_motors(speed, speed)

    def stop_motors(self, stop):
        if stop:
            self.robot.motors.stop_all_motors()


if __name__ == "__main__":
    rospy.init_node("vector")

    vector = VectorNode()
    rospy.spin()

    rospy.on_shutdown(vector.shutdown)
