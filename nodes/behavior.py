#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int16, Float64, Bool

import anki_vector

from anki_vector_ros.msg import Dist, Pose, Color


class Behavior:
    def __init__(self, robot):
        self.robot = robot

        # If we need any of the following to have responses (e.g. success/failure indicator),
        # we can transition them into services instead of topics

        self.drive_charger_sub = rospy.Subscriber(
            "/behavior/drive_charger", Bool, self.drive_charger
        )
        self.drive_straight_sub = rospy.Subscriber(
            "/behavior/drive_straight", Dist, self.drive_straight
        )
        self.find_faces_sub = rospy.Subscriber(
            "/behavior/find_faces", Bool, self.find_faces
        )
        self.look_in_place_sub = rospy.Subscriber(
            "/behavior/look_in_place", Bool, self.look_in_place
        )

        # Skipped go_to_object(), pop_a_wheelie(), roll_cube() and pickup_object()
        # need to find a way to represent LightCube

        self.pose_sub = rospy.Subscriber("/behavior/go_to_pose", Pose, self.go_to_pose)
        self.place_object_sub = rospy.Subscriber(
            "/behavior/place_object_ground", Int16, self.place_object_ground
        )
        self.roll_visible_cube_sub = rospy.Subscriber(
            "/behavior/roll_visible_cube", Bool, self.roll_visible_cube
        )
        self.say_text_sub = rospy.Subscriber(
            "/behavior/say_text", String, self.say_text
        )
        self.eye_color_sub = rospy.Subscriber(
            "/behavior/eye_color", Color, self.set_eye_color
        )

        # Note: These are configured to degrees currently
        self.head_angle_sub = rospy.Subscriber(
            "/behavior/head_angle", Float64, self.set_head_angle
        )
        self.turn_sub = rospy.Subscriber(
            "/behavior/turn_in_place", Float64, self.turn_in_place
        )

        self.lift_height_sub = rospy.Subscriber(
            "/behavior/lift_height", Float64, self.set_lift_height
        )
        self.turn_face_sub = rospy.Subscriber(
            "/behavior/turn_face", Int16, self.turn_towards_face
        )

    def drive_charger(self, bool_val):
        # True to drive on, false to drive off
        if bool_val:
            self.robot.behavior.drive_on_charger()
        else:
            self.robot.behavior.drive_off_charger()

    def drive_straight(self, msg):
        self.robot.behavior.drive_straight(msg.distance, msg.speed)

    def find_faces(self, find):
        if find:
            # This function returns a value, but it appears to be
            # a simple success/failure
            self.robot.behavior.find_faces()

    def look_in_place(self, look):
        if look:
            self.robot.behavior.look_around_in_place()

    def go_to_pose(self, pose):
        pose_props = dict()
        for attr in dir(pose):
            pose_props[attr] = getattr(pose, attr)
        pose_obj = anki_vector.util.Pose(**pose_props)

        self.robot.behavior.go_to_pose(pose_obj)

    def place_object_ground(self, retries):
        self.robot.behavior.place_object_on_ground_here(retries)

    def roll_visible_cube(self, roll):
        if roll:
            self.robot.behavior.roll_visible_cube()

    def say_text(self, text):
        self.robot.behavior.say_text(text)

    def set_eye_color(self, color):
        self.robot.behavior.set_eye_color(color.hue, color.saturation)

    def set_head_angle(self, angle):
        angle_obj = anki_vector.util.Angle(degrees=angle)
        self.robot.behavior.set_head_angle(angle_obj)

    def set_lift_height(self, height):
        self.robot.behavior.set_lift_height(height)

    def turn_in_place(self, angle):
        angle_obj = anki_vector.util.Angle(degrees=angle)
        self.robot.behavior.turn_in_place(angle_obj)

    def turn_towards_face(self, face_id):
        self.robot.behavior.turn_towards_face(face_id)
