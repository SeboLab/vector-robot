#!/usr/bin/env python3
from time import sleep

import rospy
from rospy import Publisher
from std_msgs.msg import String, Float32
from anki_vector_ros.msg import RobotStatus

import random
import speech_recognition as sr

"""
Q+A demo interaction with Vector. Requires internet connection.
"""


class MathDemoNode:
    def __init__(self):

        print("Setting up publishers")
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=3)
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)
        self.spin_pub = Publisher("/behavior/turn_in_place", Float32, queue_size=1)
        self.recognizer = sr.Recognizer()

        sleep(0.1)

        self.init_drive()
        sleep(3.0)

        correct = 0
        num_qs = 4
        for i in range(num_qs):
            if self.ask_question():
                correct += 1

        self.speech_pub(
            f"You got {correct} out of {num_qs} questions correct. Nice work!"
        )

    def init_drive(self):
        self.anim_trig_pub.publish("DriveStartHappy")
        self.speech_pub.publish("Let's do some math!")
        sleep(0.8)
        self.spin_pub.publish(6.28)

    def recognize_audio(self):
        with sr.Microphone() as source:
            print("[AUDIO] Awaiting audio")
            self.recognizer.adjust_for_ambient_noise(source)
            sleep(0.5)
            audio = self.recognizer.listen(source, timeout=2)

        try:
            text = self.recognizer.recognize_google(audio)
            print('[AUDIO] User said "', text + '"')
            return text
        except sr.UnknownValueError:
            print("[ERROR] Couldn't understand audio")
            self.speech_pub.publish("Could you say that again?")
            return None
        except sr.RequestError as e:
            print("[ERROR] Couldn't request results from Google {0}".format(e))
            return None

    def generate_question(self):
        operators = ["+", "times"]
        a = random.randint(0, 10)
        b = random.randint(0, 10)
        op = random.choice(operators)

        if op == "times":
            ans = a * b
        else:
            ans = a + b
        operation = f"{a} {op} {b}"

        return (operation, ans)

    def ask_question(self):
        operation, ans = self.generate_question()
        question = f"What is {operation}?"
        self.speech_pub.publish(question)
        text = None
        while True:
            self.anim_pub.publish("anim_rtmotion_lookup_01")
            text = self.recognize_audio()

            try:
                if text is not None:
                    user_ans = int(text)
                    break
            except ValueError:
                self.speech_pub.publish(
                    "I don't think that's a number. Let's try again!"
                )
                sleep(3.0)
                self.speech_pub.publish(question)

        if user_ans == ans:
            self.anim_pub.publish("anim_onboarding_reacttoface_happy_01_head_angle_20")
            self.speech_pub.publish(f"Yes! {operation} is {ans}. Good job.")
            sleep(8.0)
            return True
        else:
            self.anim_pub.publish("anim_rtp_blackjack_playerno_01")
            self.speech_pub.publish(f"Sorry, {operation} is {ans}.")
            sleep(4.0)
            return False


if __name__ == "__main__":
    rospy.init_node("vector_math_demo")
    rospy.wait_for_message("/status", RobotStatus)

    MathDemoNode()
