from time import sleep

import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import RobotStatus, Response
from std_msgs.msg import String, Bool


class PromptReceiver:
    """Extendable interface to build user input receiver nodes.

    __init__ should contain a Subscriber with a callback that leads to an animation trigger.
    Animation trigger should disable idling and set reacting to True

    By default, reacting is set to false when a SayTextResponse is received, though this
    may be superceded by overloading process_behavior_response()
    """

    def __init__(self):
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.idle_pub = Publisher("/labdemo/idle", Bool, queue_size=1)
        self.prompt_pub = Publisher("/labdemo/prompt", Bool, queue_size=1)

        Subscriber("/labdemo/prompt", Bool, self.process_prompting)
        Subscriber("/behavior/response", Response, self.process_behavior_response)

        # Indicates if Vector currently should be receptive to prompt triggers
        self.prompting = True

        self.reacting = False

    def process_prompting(self, msg):
        self.prompting = msg.data

    def process_behavior_response(self, resp_msg):
        if (
            resp_msg.type == "SayTextResponse"
            and self.reacting
            and resp_msg.status == 1
        ):
            sleep(2.0)
            # Animation sequence has terminated
            self.idle_pub.publish(True)
            self.prompt_pub.publish(True)
            self.reacting = False


def create_node(name, receiver):
    rospy.init_node(name)
    rospy.wait_for_message("/status", RobotStatus)
    receiver()
    rospy.spin()
