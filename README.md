# METR4202-Project
Github repository link:

https://github.com/git-von/METR4202-Project/tree/master/sg90_controller/launch

Catkin workspace packages used to control a Dynamixel servo-actuated arm in a pick and place application. Transformations and pose are found via utilisation of Aruco tags, the Aruco detect library, and Ximea ros cam library. Gripper actuation is achieved through the use of the pigpio library.

## Installation

Refer to the Aruco Detect and Ximea ros cam Githubs to install these dependencies. Refer to the following link:

https://abyz.me.uk/rpi/pigpio/download.html

For detail of Pigpio installation instruction.

To install the METR4202-Project packages, git clone the repository into the catkin_ws src folder.

## Usage
To launch the pick and place program execute the following commands in any directory:

```python
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

#
