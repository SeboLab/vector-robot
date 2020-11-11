#!/usr/bin/env python3
from rospy import Subscriber
from std_msgs.msg import String, Int16


class Media:
    def __init__(self, robot):
        self.robot = robot

        # Animations
        # Could add list of animation names as topic, but not necessary for dev use
        self.anim_sub = Subscriber("/anim/play", String, self.play_anim)
        self.anim_trigger_sub = Subscriber(
            "/anim/play_trigger", String, self.play_anim_trigger
        )

        # Audio
        self.audio_sub = Subscriber("/audio/play", String, self.play_wav)
        self.audio_vol_sub = Subscriber("/audio/vol", Int16, self.set_vol)
        self.audio_vol = 100

    def play_anim(self, str_data):
        self.robot.anim.play_animation(str_data.data)

    def play_anim_trigger(self, str_data):
        self.robot.anim.play_animation_trigger(str_data.data)

    def play_wav(self, str_data):
        self.robot.audio.stream_wav_file(str_data.data, self.audio_vol)

    def set_vol(self, vol):
        self.audio_vol = vol.data
