from time import sleep

from std_msgs.msg import Bool, Int16, String, Float32
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import LightCube, Color, RobotStatus, Dist


class CubeRoutine:
    def __init__(self):
        self.anim_pub = Publisher("/anim/play", String, queue_size=1)
        self.anim_trig_pub = Publisher("/anim/play_trigger", String, queue_size=1)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=1)
        self.light_pub = Publisher("/cube/lights", Color, queue_size=1)
        self.light_flash_pub = Publisher("/cube/flash_lights", Bool, queue_size=1)
        self.stop_light_pub = Publisher("/cube/lights_off", Bool, queue_size=1)
        self.pickup_pub = Publisher("/behavior/pickup_object", Int16, queue_size=5)
        self.lift_pub = Publisher("/behavior/lift_height", Float32, queue_size=1)
        self.angle_pub = Publisher("/behavior/head_angle", Float32, queue_size=1)
        self.drive_pub = Publisher("/behavior/drive_straight", Dist, queue_size=1)

        self.phase = 0
        self.prev_moving = False
        self.heading_to_cube = False

        Subscriber("/cube/info", LightCube, self.process_cube_state)
        Subscriber("/status", RobotStatus, self.process_robot_status)

        self.anim_pub.publish("anim_lookinplaceforfaces_keepalive_short")
        self.speech_pub.publish("Can you show me the cube?")
        sleep(1.0)

        # Set head to easily see the cube
        self.angle_pub.publish(0.2)
        self.light_flash_pub.publish(True)

    def process_cube_state(self, cube_msg):
        if cube_msg.is_moving:
            if self.phase == 0:
                # User has picked up the cube, about to set it down
                self.phase = 1
            self.stop_light_pub.publish(True)
        elif self.phase == 1:
            sleep(0.2)
            if cube_msg.is_visible and not self.heading_to_cube:
                self.light_pub.publish(Color(0, 255, 0))
                self.speech_pub.publish("Ooh I see it! Now I'm picking it up!")
                self.pickup_pub.publish(cube_msg.object_id)
                self.heading_to_cube = True
            elif self.prev_moving:
                self.light_pub.publish(Color(255, 0, 0))
                self.speech_pub.publish("I can't see the cube!")
                self.heading_to_cube = False

        elif self.phase == 2:
            self.lift_pub.publish(0.0)
            sleep(2.0)
            self.drive_pub.publish(Dist(-80, 30))
            self.stop_light_pub.publish(True)
            sleep(4.0)
            self.phase = 3

        self.prev_moving = cube_msg.is_moving

    def process_robot_status(self, status_msg):
        if status_msg.is_carrying_block and self.phase == 1:
            self.phase = 1.5
            self.anim_pub.publish("anim_eyecontact_smile_01_head_angle_40")
            self.speech_pub.publish("Hooray! We did it!")
            self.light_pub.publish(Color(0, 0, 255))
            sleep(4.0)
            self.phase = 2
