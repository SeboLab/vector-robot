#!/usr/bin/env python3
"""Detect when Vector has been petted and react accordingly."""
import random
from time import sleep

from rospy import Subscriber, Publisher
from anki_vector_ros.msg import Color, LightCube, RobotStatus, Response, Dist
from std_msgs.msg import Int16, Bool, Float32

from prompt_receiver import PromptReceiver, create_node


class CubeReceiver(PromptReceiver):
    def __init__(self):
        super().__init__()

        self.light_pub = Publisher("/cube/lights", Color, queue_size=1)
        self.pickup_pub = Publisher("/behavior/pickup_object", Int16, queue_size=1)
        self.wheelie_pub = Publisher("/behavior/wheelie", Int16, queue_size=1)
        self.stop_light_pub = Publisher("/cube/lights_off", Bool, queue_size=1)
        self.lift_pub = Publisher("/behavior/lift_height", Float32, queue_size=1)
        self.drive_pub = Publisher("/behavior/drive_straight", Dist, queue_size=1)

        Subscriber("/cube/info", LightCube, self.process_cube_state)
        Subscriber("/status", RobotStatus, self.process_robot_status)
        Subscriber("/behavior/response", Response, self.process_behavior_response)

        self.cube_initial_pickup = False
        self.reacting = False
        self.wheelie = random.choice([True, False])
        self.prev_moving = False
        self.lifting_cube = False

    def process_cube_state(self, cube_msg):
        if cube_msg.is_moving:
            print("Cube moving!")
            self.idle_pub.publish(False)
            if not self.cube_initial_pickup:
                # User has picked up the cube, about to set it down
                self.cube_initial_pickup = True
            self.stop_light_pub.publish(True)
        elif self.prompting and not self.reacting:
            sleep(0.2)
            if cube_msg.is_visible:
                self.cube_react(cube_msg.object_id)

            elif self.prev_moving:
                self.light_pub.publish(Color(255, 0, 0))
                self.speech_pub.publish("I can't see the cube!")
                self.idle_pub.publish(True)

        self.prev_moving = cube_msg.is_moving

    def cube_react(self, cube_id):
        self.idle_pub.publish(False)
        self.prompt_pub.publish(False)
        self.reacting = True
        sleep(1.0)
        self.light_pub.publish(Color(0, 255, 0))

        if self.wheelie:
            self.speech_pub.publish("Ooh I see it! Check this out!")
            self.wheelie_pub.publish(cube_id)
        else:
            self.speech_pub.publish("Ooh I see it! Now I'm picking it up!")
            self.pickup_pub.publish(cube_id)

    def process_robot_status(self, status_msg):
        if status_msg.is_carrying_block and self.reacting and not self.lifting_cube:
            # Prevent this phase from occurring again without putting the cube down
            self.lifting_cube = True

            self.anim_pub.publish("anim_eyecontact_smile_01_head_angle_40")
            self.speech_pub.publish("Hooray! We did it!")
            self.light_pub.publish(Color(0, 0, 255))
            sleep(4.0)

            # Put down the cube
            self.lift_pub.publish(0.0)
            sleep(2.0)
            self.drive_pub.publish(Dist(-80, 30))

    def process_behavior_response(self, resp_msg):
        if not self.reacting:
            return

        # Error has occurred while performing reaction
        if (
            resp_msg.type in ("PickupObjectResponse", "PopAWheelieResponse")
            and resp_msg.result != 0
        ):
            self.speech_pub.publish(
                "Oh no, I've lost the cube! Can you show me the cube again?"
            )
            self.idle_pub.publish(False)
            self.prompt_pub.publish(True)
            sleep(6.0)
            self.end_react()

        elif (
            resp_msg.type in ("PopAWheelieResponse", "DriveStraightResponse")
            and resp_msg.result == 0
        ):
            if resp_msg.type == "PopAWheelieResponse":
                # Wait for robot to right itself
                sleep(8.5)

            self.stop_light_pub.publish(True)
            self.end_react()

    def end_react(self):
        self.cube_initial_pickup = False
        self.stop_light_pub.publish(True)
        self.reacting = False
        self.wheelie = random.choice([True, False])
        self.lifting_cube = False

        self.idle_pub.publish(True)
        self.prompt_pub.publish(True)


if __name__ == "__main__":
    create_node("cube_detector", CubeReceiver)
