#!/usr/bin/env python3
"""Detect when Vector has been petted and react accordingly."""

from rospy import Subscriber
from anki_vector_ros.msg import Touch

from prompt_receiver import PromptReceiver, create_node


class PetReceiver(PromptReceiver):
    def __init__(self):
        super().__init__()

        Subscriber("/touch", Touch, self.on_touch)

        self.last_touch_status = False

    def on_touch(self, touch_msg):
        if not touch_msg.is_being_touched:
            self.last_touch_status = False
            return

        if self.prompting and not self.reacting and not self.last_touch_status:
            # If continuous petting, don't say phrase
            self.pet_react()

        self.last_touch_status = True

    def pet_react(self):
        self.idle_pub.publish(False)
        self.prompt_pub.publish(False)
        self.reacting = True
        self.anim_pub.publish("anim_petting_lvl1_01")
        self.speech_pub.publish("Ooh, that feels good!")


if __name__ == "__main__":
    create_node("pet_detector", PetReceiver)
