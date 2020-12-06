from time import sleep

from std_msgs.msg import Bool, Int16, String
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import LightCube, Color, RobotStatus


class CubeRoutine:
    def __init__(self):
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.light_pub = Publisher("/cube/lights", Color, queue_size=1)
        self.light_flash_pub = Publisher("/cube/flash_lights", Bool, queue_size=1)
        self.stop_light_pub = Publisher("/cube/lights_off", Bool, queue_size=1)
        self.pickup_pub = Publisher("/behavior/pickup_object", Int16, queue_size=1)
        self.drop_pub = Publisher("/behavior/place_object_ground", Int16, queue_size=1)

        self.phase = 0
        self.prev_moving = False
        self.heading_to_cube = False

        Subscriber("/cube/info", LightCube, self.process_cube_state)
        Subscriber("/status", RobotStatus, self.process_robot_status)

        self.speech_pub.publish("Can you show me the cube?")
        sleep(1.0)
        self.light_flash_pub.publish(True)

    def process_cube_state(self, cube_msg):
        if cube_msg.is_moving:
            if self.phase == 0:
                # User has picked up the cube, about to set it down
                self.phase = 1
            self.stop_light_pub.publish(True)
        elif self.phase == 1:
            sleep(0.1)
            if cube_msg.is_visible and not self.heading_to_cube:
                self.light_pub.publish(Color(0, 255, 0))
                self.speech_pub.publish("Ooh I see it! Now picking it up!")
                self.pickup_pub.publish(cube_msg.object_id)
                self.heading_to_cube = True
            elif self.prev_moving:
                self.light_pub.publish(Color(255, 0, 0))
                self.speech_pub.publish("I can't see the cube!")
                self.heading_to_cube = False

        if self.phase == 2:
            self.drop_pub.publish(cube_msg.object_id)
            self.phase == 3
            sleep(1.0)
            self.stop_light_pub.publish(True)

        self.prev_moving = cube_msg.is_moving

    def process_robot_status(self, status_msg):
        if status_msg.is_carrying_block and self.phase != 2:
            self.speech_pub.publish("Hooray! We did it!")
            self.light_pub.publish(Color(0, 0, 255))
            sleep(3.0)

            self.phase = 2
