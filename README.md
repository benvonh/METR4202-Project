# METR4202-Project
Github repository link:

https://github.com/git-von/METR4202-Project/tree/master/sg90_controller/launch

# Description

Catkin workspace packages used to control a Dynamixel servo-actuated arm, with 4 active degrees of freedom in a pick and place application. Utilisation of Aruco tags, the Aruco detect library, Ximea ros cam library, and the ros package tf2 are essential for the function of this project. Aruco tags were chosen as, in combination with the Aruco Detect library, the transformation from the camera can be found - thus, the transformation from the camera to two different fiducials can be fed into tf2, in order to retrieve the transform from one fiducial to the other. Additionally, the large range of unique Aruco fiducial IDs enable the software to discern between different objects. The Ximea Ros Cam library was chosen as it provides the capability to stream camera to various topics. Gripper actuation is achieved through the use of an SG90 servo and the pigpio library. Pigpio was chosen as it is one of the smoothest ways to move small servos such as the SG90.

## Installation

Refer to the Aruco Detect and Ximea ros cam Githubs to install these dependencies. Refer to the following link:

https://abyz.me.uk/rpi/pigpio/download.html

For detail of Pigpio installation instruction.

To install the METR4202-Project packages, git clone the repository into the catkin_ws src folder.

## Usage
To launch the pick and place program execute the following commands in any directory:

```python
# Launches the daemon that is essential for pigpio function
sudo pigpiod

# Launch the controller for Dynamixel servos
roslaunch dynamixel_controller dynamixel_controller.launch

# Launch the Gripper controller
roslaunch sg90_controller gripper.launch

# Launch the Ximea ros cam example launch file 
# As well as Aruco_detect.launch
roslaunch planner camera.launch

# Launches the planner.py file which is the state machine
# that controls the whole system
roslaunch planner planner.launch
```

# Package Overview
dynamixel_controller:
  Acts as an intermediary between desired pose of the end-effector to joint angles through analytical inverse kinematics.

dynamixel_interface:
  CSIRO package for control of the Dynamixel servos.
  https://github.com/csiro-robotics/dynamixel_interface

image_pipeline:
  ROS package used to calibrate camera.
  https://github.com/ros-perception/image_pipeline

pigpio_master:
  Additional library referenced by pigpio functionality.

planner:
  High-level state machine to control the entire system functionlity.

plz_plz_description:
  Package detailing URDF info and containting launch files for rviz and gazebo models of the design.

sg90_controller:
  package containing gripper servo node which closes/opens based on messages from the planner.

ximea_ros_cam:
  CSIRO package for subscribing to sensor_msgs/Image.
  https://github.com/wavelab/ximea_ros_cam
  
# Credits
Benjamin Von-Snarski; 45287008
Xavier Spinella; 45328107
Harrison Brown; 45821907
Benedict Panizza; 45853195
Kurt Talu;
