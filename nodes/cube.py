#!/usr/bin/env python3

import rospy
from rospy import Publisher
from anki_vector_ros.msg import LightCube as LightCubeMsg
from anki_vector_ros.msg import Pose
from sensors import populate_message

"""
Module to control Vector's light cube
"""


class LightCube:
    def __init__(self, robot, rate):
        self.robot = robot
        self.rate = rate

        self.robot.world.connect_cube()
        self.cube_info_pub = Publisher("/cube/info", LightCubeMsg, queue_size=1)

        # TODO: create subscribers for setting lights
        # TODO: suppress lightcube warnings
        self.publish_cube()

    def publish_cube(self):
        while not rospy.is_shutdown():
            cube = self.robot.world.connected_light_cube
            if cube is not None:
                msg = populate_message(LightCubeMsg(), cube)
                if cube.pose is not None:
                    msg.pose = populate_message(
                        Pose(), cube.pose.to_proto_pose_struct()
                    )

                self.cube_info_pub.publish(msg)

            self.rate.sleep()
