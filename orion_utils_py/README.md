# 🤖 ORION Python Utilities Package

## 🌟 Overview

This package is oriented to common utilities made with **Python** and **rclpy** for the ORION robot.

---

## 📝 License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Authors**: Daniel Felipe López Escobar, Miguel Ángel Gonzlez Rodriguez, and Alejandro Bermúdez.  
The ORION Commons packages have been tested under [ROS](https://www.ros.org/) **Jazzy** distribution.

---

## 📚 Table of Contents

- [💻 Nodes](#-nodes)
- [⚠️ Troubleshooting](#️-troubleshooting)

## 💻 Nodes

### laser_filter

Node oriented to filter the four corners of the robot based on the position of the LIDAR.
You can run it with:

~~~bash
ros2 run orion_utils_py laser_filter
~~~

## ⚠️ Troubleshooting

### Ranges not filter the columns

In case the position of your LIDAR differs from the one mentioned in the Wiki or in the robot_description, you can manually changes the ranges as it is managed as a parameter.

In the case of a execution, you can use:

~~~bash
ros2 run your_package your_node \
  --ros-args -p filter_ranges:="[0.5, 1.0, 2.2, 2.6, 3.7, 4.1, 5.3, 5.7]"
~~~

In the case of a Python Launch, you can use:

~~~bash
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='your_node',
            name='laser_filter',
            parameters=[{
                'filter_ranges': [0.5, 1.0, 2.2, 2.6, 3.7, 4.1, 5.3, 5.7]
            }]
        )
    ])
~~~
