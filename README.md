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

* `/behavior/drive_charger`: Receives a `Bool` message to trigger driving on or off Vector's charger. A `True` value makes the robot drive on its charger, while a `False` value makes it drive off its charger. **[untested]**
* `/behavior/drive_straight`: Receives a `Dist` message, making the robot drive straight for the specified distance and speed
* `/behavior/find_faces`:
* `/behavior/look_in_place`:
* `/behavior/go_to_pose`: 
* `/behavior/place_object_ground`
* `/behavior/roll_visible_cube`
* `/behavior/say_text`
* `/behavior/eye_color`
* `/behavior/head_angle`
* `/behavior/turn_in_place`
* `/behavior/lift_height`
* `/behavior/turn_face`
* `/anim/play`: Plays an animation, via a `String` message containing an animation name
* `/anim/play_trigger`: Plays an animation trigger, via a `String` message containing an animation trigger name
* `/audio/play`: Receives a `String` message containing the path of a `.wav` file and plays it
* `/audio/vol`
* `/motors/head`
* `/motors/lift`
* `/motors/wheels`
* `/motors/stop`
* `/screen/color`
* `/screen/image`
* `/screen/display_duration`


## Custom Messages