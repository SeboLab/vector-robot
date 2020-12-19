from time import sleep

"""Prompt Vector for user behavior when idling."""

import rospy
from rospy import Publisher
from std_msgs.msg import String, Float32
from anki_vector_ros.msg import RobotStatus


class Prompts:
    """Controller for prompting user behavior."""

    def __init__(self):
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.lift_pub = Publisher("/behavior/lift_height", Float32, queue_size=1)

    def prompt_fistbump(self):
        self.lift_pub.publish(0.0)
        self.speech_pub.publish("Give me a fist bump!")
        sleep(1.0)
        self.lift_pub.publish(0.6)


if __name__ == "__main__":
    rospy.init_node("vector_prompts")
    rospy.wait_for_message("/status", RobotStatus)
    Prompts()
    rospy.spin()
