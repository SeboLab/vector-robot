cmake_minimum_required(VERSION 2.8.3)
project(anki_vector_ros)


################################################
## Declare ROS messages, services and actions ##
################################################

find_package(catkin REQUIRED COMPONENTS 
   roscpp 
   rospy 
   std_msgs 
   geometry_msgs
   message_generation
)

## Generate messages in the 'msg' folder
add_message_files (
   FILES
   Dist.msg
   Color.msg
   Pose.msg
   Proximity.msg
   RobotStatus.msg
   Touch.msg
   Drive.msg
   Face.msg
   ImageRect.msg
   Object.msg
   LightCube.msg
   ColorProfile.msg
   Response.msg
)

generate_messages (
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package()
catkin_package(CATKIN_DEPENDS message_runtime)


## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   Line.srv
# )

## Generate actions in the 'action' folder
#add_action_files(
 #  FILES
#   Action1.action
#   Action2.action
# )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############



#############
## Testing ##
#############
