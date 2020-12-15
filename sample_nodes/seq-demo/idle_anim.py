#!/usr/bin/env python3

from rospy import Publisher, Subscriber
from anki_vector_ros.msg import Proximity
from std_msgs.msg import String
from anki_vector_ros.msg import Drive
import random
from time import sleep
import rospy
from std_msgs.msg import Bool
import math
import numpy
from threading import Event
import time

"""
Idle animation that makes Vector turn and move forward random amounts.
Vector will stay within a range of its starting position.
It will also avoid obstacles by using the proximity sensor.
"""

angle_multiplier = 120


class IdlePetAnimation:
    def __init__(self):
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.motor_drive_pub = Publisher("/motors/wheels", Drive, queue_size=1)
        self.motor_stop_pub = Publisher("motors/stop", Bool, queue_size=1)
        self.proximity_sub = Subscriber("/proximity", Proximity, self.proxim_callback)

        self.done = False
        self.coords = numpy.array((0.0, 0.0))
        self.theta = 0
        self.path_is_blocked = Event()

        self.animate()

    def animate(self):
        while not self.done:

            # turns a random angle
            turn_duration = random.random() + .6
            self.motor_drive_pub.publish(100, -100, 0, 0)
            self.theta += turn_duration * angle_multiplier
            sleep(turn_duration)
            self.motor_stop_pub.publish(True)

            sleep(random.random() * 3)

            # simulates where it will end up if it moves forward a random distance
            time_moved = random.random() + 1
            sim_dest = numpy.array((0.0, 0.0))
            sim_dest[0] = (
                    self.coords[0] + math.cos(math.radians(self.theta)) * time_moved
            )
            sim_dest[1] = (
                    self.coords[1] + math.sin(math.radians(self.theta)) * time_moved
            )
            sim_dist = numpy.linalg.norm(sim_dest)

            # vector only moves if it won't end up too far from the origin
            if sim_dist < 5:
                self.motor_drive_pub.publish(100, 100, 0, 0)
                start_time = time.time()
                self.path_is_blocked.wait(time_moved)

                # if it stopped because an obstacle
                if self.path_is_blocked.isSet():
                    time_passed = time.time() - start_time
                    self.coords[0] += math.cos(math.radians(self.theta)) * time_passed
                    self.coords[1] += math.sin(math.radians(self.theta)) * time_passed
                else:
                    self.coords = sim_dest

                self.motor_stop_pub.publish(True)

                sleep(random.random() * 2)

    def proxim_callback(self, proxim):
        print(proxim.distance)
        if proxim.distance < 50:
            self.path_is_blocked.set()
        else:
            self.path_is_blocked.clear()

if __name__ == "__main__":
    rospy.init_node("idle_anim_test")
    IdlePetAnimation()
