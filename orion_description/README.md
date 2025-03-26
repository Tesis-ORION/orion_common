# orion_description

## Overview

Package oriented to the deployment of the robot description by using URDF/Xacro files.

## License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Author**: Daniel Felipe López Escobar.

The *orion_description* package has been tested under [ROS](https://www.ros.org/) Jazzy.

## Installation

...

## Usage

### Model visualization:

You can visualize the */robot_description* of the robot with the [model_vis.launch.py](/orion_description/launch/model_vis.launch.py), which provide a visualization window with RViz and the usage of the **robot_state_publisher_gui**.

~~~bash
# Basic usage:
# ros2 launch orion_description model_vis.launch.py
# Additional arguments:
#   camera : Can be 'astra_s', 'a010' or 'os30a'.
#   g_mov : Boolean (true/false) to use g_mov module when using 'a010' depth cam.
#   rasp : Whether to use 'rpi4' or 'rpi5', this will imply a change in the sound hardware.
#   servo : Boolean (true/false) to indicate if use servo arms
#   
ros2 launch orion_description model_vis.launch.py camera:=os30a 
~~~

### Robot State Publisher:

To launch the */robot_description* of the robot with the [rsp.launch.py](/orion_description/launch/rsp.launch.py), which will allow you to have the topic and the transforms of the model for simulations or raw visualizations.

~~~bash
# Basic usage:
# ros2 launch orion_description rsp.launch.py
# Additional arguments:
#   camera : Can be 'astra_s', 'a010' or 'os30a'.
#   gazebo : Boolean (true/false) to indicate the usage of GZ Sim.
#   g_mov : Boolean (true/false) to use g_mov module when using 'a010' depth cam.
#   rasp : Whether to use 'rpi4' or 'rpi5', this will imply a change in the sound hardware.
#   servo : Boolean (true/false) to indicate if use servo arms
#   
ros2 launch orion_description rsp.launch.py
~~~