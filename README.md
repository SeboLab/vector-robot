# vector-robot

ROS wrapper and startup code for Vector

## Usage

1. Install ROS and `rospy`
2. [Setup the Vector Linux SDK](https://developer.anki.com/vector/docs/install-linux.html)
3. Complete the Vector robot authentication process with `python3 -m anki_vector.configure`
4. Clone this repository into your Catkin workspace
5. If needed, install additional dependencies. Run `pip3 install -r requirements.txt`.
6. You'll also need to install `cv_bridge` from source for Python 3 with `catkin build`. [This Stack Overflow thread](https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3) is useful for doing so.
6. Install this package with `catkin_make install` (from the root of your workspace) or `catkin build anki_vector_ros`
7. Run `source ~/catkin_ws/devel/setup.bash`
8. Launch a barebones instance of the Vector node with `roslaunch launch/vector_core.launch`. You may also wish to create your own `.launch` files incorporating your custom nodes. See [`hello_world.launch`](./launch/hello_world.launch) for an example.

Note: Ensure you aren't using a VPN before connecting to Vector

## Topics

The `vector_ros` node creates a series of ROS topics for interfacing with the Anki Vector sensors and outputs. Each topic is designated to either be read from or written to. Please note that this package is currently not feature-complete, particularly for features involving tracking markers and Light Cubes, but feature requests are always appreciated!

Some of these topics send/receive custom messages instead of built-in ROS messages. For details, see [custom message definitions below](#custom-messages).

### Read-only topics

* `/accel`: `Vector3` reading of the robot's XYZ acceleration
* `/gyro`: `Vector3` reading of the robot's XYZ tilt
* `/carry_object`: integer ID of the object the robot is currently carrying (-1 if none)
* `/head_angle`: Vector's head angle (in radians)
* `/head_tracking_object`: integer ID of the object that the head is tracking to (-1 if none)
* `/wheel_speed/left`: Vector's left wheel speed in mm/sec
* `/wheel_speed/right`: Vector's right wheel speed in mm/sec
* `/lift_height`: height of Vector's lift from the ground in mm
* `/localized_object` integer ID of the object that the robot is localized to (-1 if none)
* `/pose`: `Pose` message containing information about a robot's position and rotation with respect to an origin, its pose angle, and pitch (in radians)
* `/proximity`: `Proximity` message containing information with the robot's proximity to obstacles
* `/status`: `RobotStatus` message with information about the robot's sensors and position
* `/touch`: `Touch` message with the state and raw touch value of the robot's touch sensor
* `/camera`: ROS `Image` representation of the robot's front-facing camera. Use `cv_bridge` to decode this message into a OpenCV-compatible format **[untested]**

### Write-only topics

* `/behavior/drive_charger`: Receives a `Bool` message to trigger driving on or off Vector's charger. A `true` value makes the robot drive on its charger, while a `false` value makes it drive off its charger.
* `/behavior/drive_straight`: Receives a custom `Dist` message, making the robot drive straight for the specified distance and speed
* `/behavior/find_faces`: Receives a `Bool` message with a `true` value to turn in place and look for faces
* `/behavior/look_in_place`: Receives a `Bool` message with a value of `true` to turn in place
* `/behavior/go_to_pose`: Receives a `Pose` message and goes to the specified position. Note that the `angle_z`, `angle`, and `pitch` properties are ignored; quarternion values should be used instead.
* `/behavior/place_object_ground`: **[untested]**
* `/behavior/roll_visible_cube`: Receives a `Bool` message with a `true` value to roll a LightCube in view
* `/behavior/say_text`: Receives a `String` message with text to synthesize into speech
* `/behavior/eye_color`: Receives a custom `Color` message and changes Vector's eye color accordingly
* `/behavior/head_angle`: Receives a `Float32` message and turns Vector's head to the specified angle, in radians. Range in degrees is [-22, 45], with other values clamped
* `/behavior/turn_in_place`: Receives a `Float32` message and turns Vector in place by the specified amount, in radians. Positive values turn counterclockwise, while negative values turn clockwise.
* `/behavior/lift_height`: Receives a `Float32` message and sets Vector's lift to the desired height. This is clamped between 0.0 (representing the bottom position) and 1.0 (representing the top position)
* `/behavior/turn_face`: **[needs debugging]**
* `/anim/play`: Plays an animation, via a `String` message containing an animation name
* `/anim/play_trigger`: Plays an animation trigger, via a `String` message containing an animation trigger name
* `/audio/play`: Receives a `String` message containing the absolute path of a `.wav` file and plays it. Audio format must be 8000-16025 hz, 16 bits, 1 channel.
* `/audio/vol`: Recieves an integer 0-100 and sets the audiovolume accordingly. Note that this must be sent before a message is passed onto `/audio/vol` to play a file with the set volume; it does not modify sounds that are currently playing.
* `/motors/head`: Receives a `Float32` message to set Vector's head motor speed. Positive values represent up, negative values represent down. Measured in rad/sec.
* `/motors/lift`: Receives a `Float32` message to set Vector's lift motor speed. Positive values represent up, negative values represent down. Measured in rad/sec.
* `/motors/wheels`: Receives a `Drive` message and sets the velocity of the left and right treads in mm/sec.
* `/motors/stop`: Receives a boolean value of `True` to stop all motors
* `/screen/color`: Receives a `Color` message and sets Vector's screen to the chosen color for the specified duration. Default is 5 seconds.
* `/screen/image`: Receives an `String` message containing the absolute path on an image and displays it on Vector's screen. Resizes image as necessary
* `/screen/display_duration`: Receives a `Float32` to set the display duration for colors and images on the screen. This must be set before publishing to `/screen` subtopics to take effect.


## Custom Messages

* `Dist`: specifies how Vector should drive straight
  * `distance`: `Float32` value, in mm
  * `speed`: `Float32` value, in mm/sec. Note that the maximum internal speed is 220 mm/sec.
* `Drive`
* `Pose`: represents Vector's position in the world
  * `x`: X position in mm (`Float32`)
  * `y`: Y position in mm (`Float32`)
  * `z`: Z position in mm (`Float32`)
  * `q0`, `q1`, `q2`, `q3`: quarternion values representing Vector's rotation
  * `angle_z`: rotation in the z axis in radians (`Float32`)
* `Color`: represents an RGB color combination
  * `red`: int value 0-255
  * `green`: int value 0-255 
  * `blue`: int value 0-255
* `Proximity`: a reading from Vector's proximity sensor
  * `distance`: `Float32` value, in mm
  * `found_object`: boolean value indicating if object is found by the sensor
  * `is_lift_in_fov`: boolean value indicating if Vector's lift is blocking its sensor
  * `signal_quality`: `Float32` representing likelihood of reported distance being a solid surface
  * `unobstructed`: boolean value confirming if no objects are detected
* `RobotStatus`: various boolean values representing Vector's state. Use this message to determine when all of Vector's nodes are online
  * `are_motors_moving`
  * `are_wheels_moving`
  * `is_animating`
  * `is_being_held`
  * `is_button_pressed`
  * `is_carrying_block`
  * `is_charging`
  * `is_cliff_detected`
  * `is_docking_to_marker`
  * `is_falling`
  * `is_head_in_pos`
  * `is_in_calm_power_mode`
  * `is_lift_in_pos`
  * `is_on_charger`
  * `is_pathing`
  * `is_picked_up`
  * `is_robot_moving`
* `Touch`: reading from Vector's touch sensor
  * `is_being_touched`: Vector's conclusion of if it is being touched (boolean)
  * `raw_touch_value`: integer representing the detected sensitivity from the touch sensor