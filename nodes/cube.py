#!/usr/bin/env python3

import rospy
from rospy import Publisher, Subscriber
from anki_vector.lights import Light, ColorProfile, Color
from std_msgs.msg import Bool
from time import sleep

from anki_vector_ros.msg import (
    Pose,
    LightCube as LightCubeMsg,
    ColorProfile as ColorProfileMsg,
    Color as ColorMsg,
)

from util import populate_message

"""
Module to control Vector's light cube
"""


class LightCube:
    def __init__(self, robot, rate):
        self.robot = robot
        self.rate = rate

        self.profile = ColorProfile(1.0, 1.0, 1.0)

        self.cube_info_pub = Publisher("/cube/info", LightCubeMsg, queue_size=1)
        self.profile_sub = Subscriber(
            "/cube/color_profile", ColorProfileMsg, self.set_color_profile
        )
        self.light_sub = Subscriber("/cube/lights", ColorMsg, self.set_lights)
        self.light_off_sub = Subscriber("/cube/lights_off", Bool, self.set_lights_off)
        self.light_off_sub = Subscriber("/cube/flash_lights", Bool, self.flash_lights)

        self.robot.world.connect_cube()
        sleep(0.1)
        # Return cube to default state
        if self.get_cube() is not None:
            self.get_cube().set_lights_off()

        self.publish_cube()

    def get_cube(self):
        self.robot.world.connect_cube()
        return self.robot.world.connected_light_cube

    def flash_lights(self, flash):
        if flash:
            self.robot.world.flash_cube_lights()

    def set_lights(self, color):
        color_obj = Color(rgb=(color.red, color.green, color.blue))
        self.get_cube().set_lights(Light(color_obj), self.profile)

    def set_lights_off(self, set_off):
        if set_off:
            self.get_cube().set_lights_off()

    def set_color_profile(self, profile):
        self.profile = ColorProfile(
            profile.red_multiplier, profile.green_multiplier, profile.blue_multiplier
        )

    def publish_cube(self):
        while not rospy.is_shutdown():
            cube = self.get_cube()
            if cube is not None:
                msg = populate_message(LightCubeMsg(), cube)

                if cube.pose is not None:
                    msg.pose = populate_message(
                        Pose(), cube.pose.to_proto_pose_struct()
                    )
                else:
                    msg.pose = Pose()

                self.cube_info_pub.publish(msg)

            self.rate.sleep()
