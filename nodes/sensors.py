#!/usr/bin/env python3
import rospy
from rospy import Publisher

from anki_vector_ros.msg import Pose, Proximity, RobotStatus, Touch
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Vector3


class Sensors:
    def __init__(self, robot, rate):
        self.robot = robot
        self.rate = rate

        # Info publishing
        self.accel_pub = Publisher("/accel", Vector3, queue_size=1)
        self.gyro_pub = Publisher("/gyro", Vector3, queue_size=1)
        self.carry_object_pub = Publisher("/carry_object", Int16, queue_size=1)
        self.angle_pub = Publisher("/head_angle", Float32, queue_size=1)
        self.tracking_pub = Publisher("/head_tracking_object", Int16, queue_size=1)

        # This is in mm/sec
        self.left_wheel_pub = Publisher("/wheel_speed/left", Float32, queue_size=1)
        self.right_wheel_pub = Publisher("/wheel_speed/right", Float32, queue_size=1)

        # This is in mm
        self.lift_height_pub = Publisher("/lift_height", Float32, queue_size=1)
        self.localized_pub = Publisher("/localized_object", Int16, queue_size=1)

        self.pose_pub = Publisher("/pose", Pose, queue_size=1)
        self.proximity_pub = Publisher("/proximity", Proximity, queue_size=1)
        self.status_pub = Publisher("/status", RobotStatus, queue_size=1)
        self.touch_pub = Publisher("/touch", Touch, queue_size=1)

        self.publish_sensor_feed()

    def publish_sensor_feed(self):
        while not rospy.is_shutdown():
            # Publish sensor/motor data
            self.accel_pub.publish(**convert_vector3(self.robot.accel))
            self.gyro_pub.publish(**convert_vector3(self.robot.gyro))
            self.angle_pub.publish(self.robot.head_angle_rad)
            self.tracking_pub.publish(self.robot.head_tracking_object_id)
            self.left_wheel_pub.publish(self.robot.left_wheel_speed_mmps)
            self.right_wheel_pub.publish(self.robot.right_wheel_speed_mmps)

            self.lift_height_pub.publish(self.robot.lift_height_mm)
            self.localized_pub.publish(self.robot.localized_to_object_id)
            self.carry_object_pub.publish(self.robot.carrying_object_id)

            pose_msg = populate_message(Pose(), self.robot.pose.to_proto_pose_struct())
            # Note: these values are given in radians
            pose_msg.angle = self.robot.pose_angle_rad
            pose_msg.pitch = self.robot.pose_pitch_rad
            self.pose_pub.publish(pose_msg)

            proximity_obj = self.robot.proximity.last_sensor_reading
            proximity_msg = populate_message(Proximity(), proximity_obj)
            proximity_msg.distance = proximity_obj.distance.distance_mm
            self.proximity_pub.publish(proximity_msg)

            status_msg = populate_message(RobotStatus(), self.robot.status)
            self.status_pub.publish(status_msg)

            touch_msg = populate_message(Touch(), self.robot.touch.last_sensor_reading)
            self.touch_pub.publish(touch_msg)

            self.rate.sleep()


def convert_vector3(vector3_obj):
    attr_dict = dict()
    for attr in dir(vector3_obj):
        if attr not in ("x", "y", "z"):
            continue
        attr_dict[attr] = getattr(vector3_obj, attr)

    return attr_dict


def populate_message(message, vector_obj):
    for attr in dir(vector_obj):
        if not hasattr(message, attr):
            continue
        if "__" in attr:
            continue

        setattr(message, attr, getattr(vector_obj, attr))

    return message
