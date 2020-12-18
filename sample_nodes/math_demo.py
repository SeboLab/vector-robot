#!/usr/bin/env python3

"""Q+A demo interaction with Vector. Requires internet connection."""

import random
import sys
from time import sleep

import rospy
from rospy import Publisher
from std_msgs.msg import String, Float32
from anki_vector_ros.msg import RobotStatus

import speech_recognition as sr


class MathDemoNode:
    def __init__(self):

        print("Setting up publishers")
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=3)
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)
        self.spin_pub = Publisher("/behavior/turn_in_place", Float32, queue_size=1)
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = 0.5
        self.recognizer.dynamic_energy_threshold = False
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=3)

        # Overcompensate for background noise
        self.recognizer.energy_threshold += 200

        sleep(0.1)

        self.init_drive()
        sleep(3.0)

        correct = 0
        num_qs = 4
        for _ in range(num_qs):
            if self.ask_question():
                correct += 1

        self.anim_trig_pub.publish("FetchCubeSuccess")
        self.speech_pub.publish(
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
            audio = self.recognizer.listen(source, timeout=10.0)

        try:
            text = self.recognizer.recognize_google(audio)
            print('[AUDIO] User said "', text + '"')
            return text
        except sr.WaitTimeoutError:
            return "repeat"
        except sr.UnknownValueError:
            print("[ERROR] Couldn't understand audio")
            self.speech_pub.publish("Could you say that again?")
            return None
        except sr.RequestError as e:
            print("[ERROR] Couldn't request results from Google {0}".format(e))
            return None

    def ask_question(self):
        operation, ans = generate_question()
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
                if text.strip() != "repeat":
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

        self.anim_pub.publish("anim_rtp_blackjack_playerno_01")
        self.speech_pub.publish(f"Sorry, {operation} is {ans}.")
        sleep(4.0)
        return False


def generate_question():
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


if __name__ == "__main__":
    rospy.init_node("vector_math_demo")
    rospy.wait_for_message("/status", RobotStatus)

    try:
        MathDemoNode()
    except KeyboardInterrupt:
        sys.exit(0)
