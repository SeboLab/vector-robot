#!/usr/bin/env python3
import colorsys
from rospy import Subscriber, Publisher
from std_msgs.msg import String, Int16, Float32, Bool

import anki_vector

from anki_vector_ros.msg import Dist, Pose, Color, Response


class Behavior:
    def __init__(self, robot):
        self.robot = robot

        # If we need any of the following to have responses (e.g. success/failure indicator),
        # we can transition them into services instead of topics

        self.drive_charger_sub = Subscriber(
            "/behavior/drive_charger", Bool, self.drive_charger
        )
        self.drive_straight_sub = Subscriber(
            "/behavior/drive_straight", Dist, self.drive_straight
        )
        self.find_faces_sub = Subscriber("/behavior/find_faces", Bool, self.find_faces)
        self.look_in_place_sub = Subscriber(
            "/behavior/look_in_place", Bool, self.look_in_place
        )

        self.pose_sub = Subscriber("/behavior/go_to_pose", Pose, self.go_to_pose)
        self.place_object_sub = Subscriber(
            "/behavior/place_object_ground", Int16, self.place_object_ground
        )
        self.roll_visible_cube_sub = Subscriber(
            "/behavior/roll_visible_cube", Bool, self.roll_visible_cube
        )
        self.say_text_sub = Subscriber("/behavior/say_text", String, self.say_text)
        self.eye_color_sub = Subscriber(
            "/behavior/eye_color", Color, self.set_eye_color
        )

        # Note: These are configured to radians currently
        self.head_angle_sub = Subscriber(
            "/behavior/head_angle", Float32, self.set_head_angle
        )
        self.turn_sub = Subscriber(
            "/behavior/turn_in_place", Float32, self.turn_in_place
        )

        self.lift_height_sub = Subscriber(
            "/behavior/lift_height", Float32, self.set_lift_height
        )
        self.turn_face_sub = Subscriber(
            "/behavior/turn_face", Int16, self.turn_towards_face
        )

        self.go_to_object_sub = Subscriber(
            "/behavior/go_to_object", Int16, self.go_to_object
        )
        self.wheelie_sub = Subscriber("/behavior/wheelie", Int16, self.wheelie)
        self.roll_cube_sub = Subscriber("/behavior/roll_cube", Int16, self.roll_cube)
        self.dock_cube_sub = Subscriber(
            "/behavior/dock_cube", Int16, self.dock_with_cube
        )
        self.pickup_sub = Subscriber(
            "/behavior/pickup_object", Int16, self.pickup_object
        )

        self.response_pub = Publisher("/behavior/response", Response, queue_size=10)

    def publish_response(self, resp):
        print(resp)
        type_name = str(type(resp)).split(".")[-1][:-2]
        result = resp.result.code if "result" in dir(resp) else -1
        self.response_pub.publish(Response(resp.status.code, result, type_name))

    def drive_charger(self, bool_val):
        # True to drive on, false to drive off
        if bool_val.data:
            resp = self.robot.behavior.drive_on_charger()
        else:
            resp = self.robot.behavior.drive_off_charger()
        self.publish_response(resp)

    def drive_straight(self, msg):
        dist = anki_vector.util.distance_mm(msg.distance)
        speed = anki_vector.util.speed_mmps(msg.speed)
        resp = self.robot.behavior.drive_straight(dist, speed)
        self.publish_response(resp)

    def find_faces(self, find):
        if find.data:
            # This function returns a value, but it appears to be
            # a simple success/failure
            self.robot.behavior.find_faces()

    def look_in_place(self, look):
        if look.data:
            self.robot.behavior.look_around_in_place()

    def go_to_pose(self, pose):
        pose_props = dict()
        pose_attrs = ["x", "y", "z", "q0", "q1", "q2", "q3", "origin_id"]
        for attr in dir(pose):
            if attr not in pose_attrs:
                continue

            pose_props[attr] = getattr(pose, attr)
        pose_obj = anki_vector.util.Pose(**pose_props)

        resp = self.robot.behavior.go_to_pose(pose_obj)
        self.publish_response(resp)

    def place_object_ground(self, retries):
        resp = self.robot.behavior.place_object_on_ground_here(retries.data)
        self.publish_response(resp)

    def roll_visible_cube(self, roll):
        if roll.data:
            self.robot.behavior.roll_visible_cube()

    def say_text(self, text):
        self.publish_response(self.robot.behavior.say_text(text.data))

    def set_eye_color(self, color):
        hue, sat, _ = colorsys.rgb_to_hsv(color.red, color.green, color.blue)
        resp = self.robot.behavior.set_eye_color(hue, sat)
        self.publish_response(resp)

    def set_head_angle(self, angle):
        angle_obj = anki_vector.util.Angle(radians=angle.data)
        resp = self.robot.behavior.set_head_angle(angle_obj)
        self.publish_response(resp)

    def set_lift_height(self, height):
        resp = self.robot.behavior.set_lift_height(height.data)
        self.publish_response(resp)

    def turn_in_place(self, angle):
        angle_obj = anki_vector.util.Angle(radians=angle.data)
        resp = self.robot.behavior.turn_in_place(angle_obj)
        self.publish_response(resp)

    def turn_towards_face(self, face_id):
        face = self.robot.world.get_face(face_id.data)
        resp = self.robot.behavior.turn_towards_face(face)
        self.publish_response(resp)

    def roll_cube(self, object_id):
        obj = self.robot.world.get_object(object_id.data)
        resp = self.robot.behavior.roll_cube(obj)
        self.publish_response(resp)

    def dock_with_cube(self, object_id):
        obj = self.robot.world.get_object(object_id.data)
        resp = self.robot.behavior.dock_with_cube(obj)
        self.publish_response(resp)

    def pickup_object(self, object_id):
        obj = self.robot.world.get_object(object_id.data)
        resp = self.robot.behavior.pickup_object(obj)
        self.publish_response(resp)

    def wheelie(self, object_id):
        obj = self.robot.world.get_object(object_id.data)
        resp = self.robot.behavior.pop_a_wheelie(obj)
        self.publish_response(resp)

    def go_to_object(self, object_id):
        obj = self.robot.world.get_object(object_id.data)
        distance = 0  # Can modify this as needed
        resp = self.robot.behavior.go_to_object(obj, distance)
        self.publish_response(resp)
