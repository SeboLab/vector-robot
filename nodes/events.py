#!/usr/bin/env python3
from anki_vector.events import Events

from rospy import Publisher
from anki_vector_ros.msg import Face, Object, ImageRect, Pose
from util import populate_message

"""
Module with EventHandler to publish to ROS topics
Events are taken from
https://sdk-resources.anki.com/vector/docs/generated/anki_vector.events.html
"""


class EventHandler:
    def __init__(self, robot):
        self.robot = robot
        self.face_pub = Publisher("/events/face", Face, queue_size=1)
        self.object_pub = Publisher("/events/object", Object, queue_size=1)

        # These are triggered via /behavior/find_faces and /behavior/look_in_place
        # If the camera is enabled, Vector also finds faces automatically
        self.robot.events.subscribe(
            self.on_observed_object, Events.robot_observed_face, face=True
        )
        self.robot.events.subscribe(
            self.on_observed_object, Events.robot_observed_object
        )

    def on_observed_object(self, robot, event_type, event, face=False):
        msg = Face() if face else Object()
        msg = populate_message(msg, event)

        msg.img_rect = populate_message(ImageRect(), event.img_rect)
        msg.pose = populate_message(Pose(), event.pose)

        publisher = self.face_pub if face else self.object_pub
        publisher.publish(msg)
