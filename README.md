# vector-robot

ROS wrapper and startup code for Vector

## Usage

1. Install ROS and `rospy`
2. [Setup the Vector Linux SDK](https://developer.anki.com/vector/docs/install-linux.html)
3. Complete the Vector robot authentication process with `python3 -m anki_vector.configure`
4. Clone this repository into your Catkin workspace
5. If needed, install additional dependencies. Run `pip3 install -r requirements.txt`
6. Install this package with `catkin_make install` (from the root of your workspace)
7. Run `source ~/catkin_ws/devel/setup.bash`
8. Launch a barebones instance of the Vector node with `roslaunch launch/vector_core.launch`. You may also wish to create your own `.launch` files incorporating your custom nodes
9. More coming soon...

## Topics

* `/accel`: A `Vector3` reading of the robot's XYZ acceleration
* `/gyro`: A `Vector3` reading of the robot's XYZ tilt
