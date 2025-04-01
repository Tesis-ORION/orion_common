# ORION Commons

## Overview

Package oriented to the basic usage of the ORION project, a ROS 2 differential robot oriented for Human-Robot Interaction applications.

**Keywords:** ROS 2, Differential, HRI, ROS 2 Jazzy, low-cost.

## License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Authors**: Daniel Felipe López Escobar, Miguel Ángel Gonzalez Rodriguez and Alejandro Bermudez.

The ORION Common packages have been tested under [ROS](https://www.ros.org/) Jazzy.

## Package summary

- **[orion](/orion/README.md):** Pseudo package of the robot.
- **[orion_assets](/orion_assets/README.md):** Collection of CAD files for the construction, design and assembly of the robot.
- **[orion_control](/orion_control/README.md):** Package oriented to configuring the control of the robot's motors.
- **[orion_description](/orion_description/README.md):** Package oriented to the URDF/Xacro description of the robot.

## Installation

1. Create your workspace:

    ~~~bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    ~~~

2. Install the repository of ORION common in the source directory

    ~~~bash
    cd ~/ros2_ws/src

    # For now it is on branch dev
    git clone -b dev https://github.com/Tesis-ORION/orion_common.git
    ~~~

3. Install external packages dependencies in the source

    ~~~bash
    cd ~/ros2_ws/src
    # For now also on dev branch
    git clone -b dev https://github.com/DanielFLopez1620/G-Mov_Project.git
    ~~~

4. Install all the dependencies:

    ~~~bash
    sudo apt update
    sudo apt install python3-rosdep -y
    cd ~/ros2_ws
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ~~~

5. After the installation is complete, build the package with the provided options to avoid errors with other packages in development:

    ~~~bash
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select g_mov_description orion orion_description orion_control
    source install/setup.bash
    ~~~

6. You are ready to explore the usage of the robot.
