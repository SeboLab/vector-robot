#!/usr/bin/env python3
from time import sleep
import rospy
from rospy import Publisher
from std_msgs.msg import String
from anki_vector_ros.msg import RobotStatus

"""
Sample program with continuous looped interactions with Vector, WIP.

Prompts:
    These randomly occur every 15 seconds of holding the idle animation. Upon completing
    each prompt, Vector returns to the idle animation.

    Petting:
        Animate with happiness
        Say "Can you pet my back?"
    Tuck in:
        Say "I getting sleepy... Can you cover me with my blanket?"
        Animate with going to sleep
        Animate with up to 20 sec of rest until response
    Wheelie or pick up cube:
        Animate with excitement
        Say "Can you show me the cube?"
        Flash lights on cube
        Remain in place (perhaps moving head) for up to 15 seconds since last cube movement
        (or timeout occurs)
    Fist bump:
        Animate with happiness
        Say "Give me a fist bump"

Triggers and responses:
    These responses can be triggered at any time when Vector is in an idle animation,
    except for when it is performing the sleep animation prompt. Upon their completion,
    Vector returns to the idle animation.

    Camera covered up:
        Say "Thanks for tucking me in"
        Perform animation
    Touch sensor is activated (account for noise):
        React to petting
    x-acceleration and short distance is detected (when Vector) is not
    programmed to move:
        React to fistbump
    Cube is picked up by a user, then put into Vector's view:
        Perform a wheelie or do cube pickup routine

TODO: Instead of timer-based actions, use response callbacks to determine state
"""


class LabDemoNode:
    def __init__(self):

        print("Setting up publishers")
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)

        # Wait to setup publishers
        sleep(0.5)
        print("Starting!")
        self.speech_pub.publish("Hi, my name is Vector!")
        sleep(0.5)
        self.anim_pub.publish("anim_eyepose_happy")


if __name__ == "__main__":
    rospy.init_node("labdemo")
    rospy.wait_for_message("/status", RobotStatus)

    LabDemoNode()
