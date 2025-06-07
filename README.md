# 🤖 ORION Commons

## 🌟 Overview

This repository contains essential packages for the **O**pen-source **R**obot for **I**nteraction **O**bjectives and **N**avigation, also known as **ORION Project**, a ROS 2-based differential mobile robot designed for **Human-Robot Interaction (HRI)** applications.

**Keywords:** ROS 2, Differential Robot, HRI, ROS 2 Jazzy, Low-Cost Robotics.

---

## 📝 License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Authors**: [Daniel Felipe López Escobar](https://github.com/DanielFLopez1620), [Miguel Ángel Gonzalez Rodriguez](https://github.com/miguelgonrod), and [Alejandro Bermúdez Fajardo](https://github.com/alexoberco).

The ORION Commons packages have been tested under [ROS](https://www.ros.org/) **Jazzy** distribution.

---

## 📚 Table of Contents

- [📦 Repository Summary](#-repository-summary)
- [🧩 Other Functionalities](#-other-functionalities)
- [📥 Installation](#-installation)
- [⚠️ Troubleshooting](#️-troubleshooting)

---

## 📦 Repository Summary

The repository is organized into modular ROS 2 packages:

- 🌌 **[`orion`](/orion/README.md)** 🛰️ Metapackage grouping all main components dependencies.
- 📦 **[`orion_assets`](/orion_assets/README.md)** 🗂️ CAD files for design, assembly, and construction of the robot.
- 🎮 **[`orion_control`](/orion_control/README.md)** 🧠 Configuration for controllers and hardware interfaces plugins for ROS 2 controllers.
- 🧩 **[`orion_description`](/orion_description/README.md)** 📐 URDF/Xacro description of the robot’s structure.
- 🏗️ **[`orion_base`](/orion_base/README.md)** 📦 Core logic and embedded codes of the ESP32s (mobile base and interaction) of the robot.
- 🚀 **[`orion_bringup`](/orion_bringup/README.md)** 🟢 Launch and startup configuration for the usage of the real robot.
- 🐳 **[`orion_docker`](/orion_docker/README.md)** 📦 Docker support for containerized development and deployment of the robot.
- 🧰 **[`orion_utils_py`](/orion_utils_py/README.md)** 🐍 Utility scripts in Python 3 to common applications like laser filter or simple actions.

To build you robot, review the hardware changes and learn aout the versions of the robot, do not forget to check the 📖 [`ORION Wiki`](https://github.com/Tesis-ORION/orion_common/wiki) 📔

---

## 🧩 Other Functionalities

These components provide extended capabilities for sensors, simulation, perception, and interaction:

- 💬 **[`orion_chat`](https://github.com/Tesis-ORION/orion_chat)** 🤖 Natural Language Processing interface for interacting with the robot and send commands (for example, specificy arm movement or command a velocity to the robot).

- 🧿 **[`orion_gz`](https://github.com/Tesis-ORION/orion_gz)** 🏙️ Simulation of the robot in GZ Harmonic that integrates native plugins, bridges between ROS 2 and GZ with [`ros_gz_bridge`](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge), and the integration of [`ros2_control`](https://control.ros.org/) with [`gz_ros2_control`](https://github.com/ros-controls/gz_ros2_control).

- 📄 **[`orion_tools`](https://github.com/Tesis-ORION/orion_tools)** 🔧 A collection of packages for using SLAM, Nav2 and teleoperation with the robot.

- 🌐 **[`orion_web_interface`](https://github.com/Tesis-ORION/orion_web_interface)** 🖥️ Tool that allows the control and visualization of the robot by using a [`Node.js`](https://nodejs.org/en) and [`Astro`](https://astro.build/) Web interface.

- 🎥 **[`depth_orbbec_astra`](https://github.com/Tesis-ORION/depth_orbbec_astra)** 🔵 Packages to use the ORBBEC Astra RGBD Cameras on ROS 2 Jazzy. In this project is used the [ASTRA S](https://store.orbbec.com/products/astra-s) model.

- 🎥 **[`depth_ydlidar_os30a`](https://github.com/Tesis-ORION/Depth_ydlidar_os30a)** 🔴 Package to use the [YDLIDAR OS30A](https://www.ydlidar.com/products/view/23.html) on ROS 2 Jazzy.

- 🎥 **[`depth_maixsense_a010`](https://github.com/Tesis-ORION/depth_maixsense_a010)** 🟢 Packages for the [Maixsense A010 Depth Camera](https://wiki.sipeed.com/hardware/en/maixsense/maixsense-a010/maixsense-a010.html) to work on ROS 2 Jazzy.

- 😊 **[`emotion_detector`](https://github.com/Tesis-ORION/emotion_detector)** 🧠 Emotion recognition pipeline based on computer vision and facial analysis.

---

## 📥 Installation

Let's prepare us to use the robot, this installation is required for both your PC and the robot's Raspberry Pi. However, there would be additional steps you will need to follow on the robot, more info on [orion_bringup](/orion_bringup/README.md)

For now, follow these steps to install and build the project on ROS 2 Jazzy:

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
    git clone https://github.com/Tesis-ORION/orion_common.git
    ~~~

3. Install the drivers packages for the cameras.

    ~~~bash
    cd ~/ros2_ws/src
    git clone https://github.com/Tesis-ORION/depth_maixsense_a010.git
    git clone https://github.com/Tesis-ORION/Depth_ydlidar_os30a.git
    git clone https://github.com/Tesis-ORION/depth_orbbec_astra.git
    ~~~

4. Install the [`orion_chat`](https://github.com/Tesis-ORION/orion_chat) package:

    ~~~bash
    git clone https://github.com/Tesis-ORION/orion_chat.git  
    cd orion_chat
    ./install_apt.sh
    pip install -r requirements.txt --break-system-packages
    ~~~

5. Implement the additional installs recommended on the cameras READMEs, for more info check [Maixsense A010](https://github.com/Tesis-ORION/depth_maixsense_a010), [YDLidar OS30A](https://github.com/Tesis-ORION/Depth_ydlidar_os30a) and [ORBBEC ASTRA S](https://github.com/Tesis-ORION/depth_orbbec_astra) packages.

    ~~~bash
    # --------- General 
    sudo usermod -a -G dialout $USER

    # -------- OS30A
    sudo ln -sf /lib/x86_64-linux-gnu/libdc1394.so /usr/lib/libdc1394.so.22
    
    # Edit file
    sudo nano /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp
    # Change the following lines
    # From ".hpp" to ".h" in:
    #include "tf2/convert.h"
    #include "tf2/LinearMath/Quaternion.hpp"
    #include "tf2/LinearMath/Transform.hpp"
    #include "tf2/LinearMath/Vector3.hpp"

    # --------- ASTRA S
    sudo apt install libgflags-dev nlohmann-json3-dev  \
    ros-$ROS_DISTRO-image-transport  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
    ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
    ros-$ROS_DISTRO-backward-ros libdw-dev
    ~~~

6. Install external packages dependencies in the source

    ~~~bash
    cd ~/ros2_ws/src
    git clone https://github.com/DanielFLopez1620/G-Mov_Project.git
    ~~~

7. Install all the dependencies:

    ~~~bash
    sudo apt update
    sudo apt install python3-rosdep -y
    cd ~/ros2_ws
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ~~~

8. After the installation is complete, build the package with the provided options to avoid errors with other packages in development:

    ~~~bash
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select g_mov_description orion orion_description orion_control
    source install/setup.bash
    ~~~

9. You are ready to explore the usage of the robot on this PC, now proceed with the robot [bringup](/orion_bringup/README.md)

---

## ⚠️ Troubleshooting

Explore the different packages to check solutions to common problems found during the development of the project, considering the next:

- **orion_base:** Cases in terms of the embedded codes of the ESP32, µ-ROS, electronic connections and hardware specifications.
- **orion_bringup:** In terms of the startup application of the robot and the bringup of the robot.
- **orion_control:** For problems related with the plugins for the hardware interfaces of the controllers and general params of the controllers.
- **orion_description:** Cases related with the description model, changes of macros and params of actuators/controllers.

If you present another problem, propose it on the **[Issues](https://github.com/Tesis-ORION/orion_common/issues)** of this repository.

---