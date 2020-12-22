#!/usr/bin/env python3
"""Prompt Vector for user behavior when idling."""
import random
from time import sleep

import rospy
from rospy import Publisher, Subscriber
from std_msgs.msg import String, Float32, Bool
from anki_vector_ros.msg import RobotStatus


class Prompts:
    """Controller for prompting user behavior."""

    def __init__(self):
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.lift_pub = Publisher("/behavior/lift_height", Float32, queue_size=1)
        self.light_flash_pub = Publisher("/cube/flash_lights", Bool, queue_size=1)
        self.head_angle_pub = Publisher("/behavior/head_angle", Float32, queue_size=1)
        self.idle_pub = Publisher("/labdemo/idle", Bool, queue_size=1)
        self.prompt_pub = Publisher("/labdemo/prompt", Bool, queue_size=1)

        self.idling = False
        self.prompting = True
        self.prompt_queued = False
        self.prompts = [self.prompt_fistbump, self.prompt_pet, self.prompt_cube]
        self.last_prompt = -1
        Subscriber("/labdemo/idle", Bool, self.idle_callback)
        Subscriber("/labdemo/prompt", Bool, self.prompting_callback)

    def idle_callback(self, msg):
        self.idling = msg.data

        if msg.data and not self.prompt_queued:
            # Let Vector idle for some period
            self.prompt_queued = True
            sleep(random.randint(5, 10))

            # Robot is no longer idling, no need to prompt
            if not self.idling:
                return

            # Robot is still idling, prompt user
            self.idle_pub.publish(False)
            self.prompt_pub.publish(True)
            
            prompt_i = random.randint(0, len(self.prompts) - 1)
            while prompt_i == self.last_prompt:
                prompt_i = random.randint(0, len(self.prompts) - 1)

            self.last_prompt = prompt_i
            prompt = self.prompts[prompt_i]
            prompt()

            sleep(15.0)  # Wait for user response
            self.prompt_queued = False
            # User hasn't evoked a response from Vector yet, return to idle
            if self.prompting:
                self.idle_pub.publish(True)

    def prompting_callback(self, msg):
        self.prompting = msg.data

    def prompt_fistbump(self):
        self.lift_pub.publish(0.0)
        self.speech_pub.publish("Give me a fist bump!")
        sleep(1.0)
        self.lift_pub.publish(0.6)

    def prompt_pet(self):
        self.speech_pub.publish("Do you want to pet my back?")

    def prompt_cube(self):
        self.anim_pub.publish("anim_lookinplaceforfaces_keepalive_short")
        self.speech_pub.publish("Can you show me the cube?")
        sleep(1.0)

        # Set head to easily see the cube
        self.head_angle_pub.publish(0.2)
        self.light_flash_pub.publish(True)


if __name__ == "__main__":
    rospy.init_node("vector_prompts")
    rospy.wait_for_message("/status", RobotStatus)
    Prompts()
    rospy.spin()
