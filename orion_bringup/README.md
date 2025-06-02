# 🤖 ORION Commons

## 🌟 Overview

This repository contains teh bringup and start up of the ORION robot. Before using this package, do not forget to upload the codes on the two ESP32 on [orion_base](/orion_base/README.md)

---

## 📝 License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Authors**: Daniel Felipe López Escobar, Miguel Ángel Gonzalez Rodriguez, and Alejandro Bermúdez.

The ORION Commons packages have been tested under [ROS](https://www.ros.org/) **Jazzy** distribution.

---

## 📚 Table of Contents

- [📝 Udev rules set up](#-udev-rules-set-up)
- [🚀 Launch files](#-udev-rules-set-up)
- [🚀 Setup of bringup on Startup](#-setup-of-bringup-on-startup)
- [⚠️ Troubleshooting](#️-troubleshooting)

---

## 📝 Udev rules set up

As we are using multiple USB devices which may change of port or even the USB devices may be different, for example, depending on your camaras selection. This is the reason we require to implement **udev** rules.

### General considerations

Begin by connecting each device (only one at a time) and reading its attributes:

~~~bash
ls /dev | grep USB
# With the USB device found use
udevadm info --name=/dev/ttyUSB0 --attribute-walk
~~~

A good start is to focus on the attributes like **idVendor**, **idProduct**, **serial** and **product**. If you do not find them, you can use **grep**.

~~~bash
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep idVendor
~~~

In case you note the attributes selected repeat with others, search for others. And as last resource, you can use environmental attributes, like the **ID_PATH**, this is not always recommended as if the connections change it will be different later.

~~~bash
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep ID_PATH
~~~

Once you have decided the attributes and elements, you can create a file for these rules and add the content:

~~~bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
~~~

~~~bash
# Inside of the rules file
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttyESP32_1"

~~~

A demo file you can use to replicate your rules is [example_udev.rules](/orion_bringup/example_udev.rules):

~~~rules
UBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttyLD19"
SUBSYSTEM=="tty", ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3.1:1.0", SYMLINK+="ttyESP32_1"
SUBSYSTEM=="tty", ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3.3:1.0", SYMLINK+="ttyESP32_2"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", ATTRS{serial}=="202206 DB225C", SYMLINK+="ttyA010"
SUBSYSTEM=="usb", ATTR{idProduct}=="0402", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="astra_s"
~~~

### Comments based on components

- **LD19:** Its rules were based on the [ldrobot-lidar-ros2](https://github.com/Myzhar/ldrobot-lidar-ros2/blob/devel/rules/ldlidar.rules) package of Myzhar. This apply for the youyeetuu LD19 LIDAR and worked on 3 different LIDARs. However, do not forget to validate the attribues.

- **ORBBEC Astra S:** Its rules were taken from [depth_orbbec_astra](https://github.com/Tesis-ORION/depth_orbbec_astra/blob/main/orbbec_camera/scripts/99-obsensor-libusb.rules) repository.

- **MaixSense A010:** when connecting this device, you may notice it gives two USB devices, one for the device itself and another one for debugging, keep this in mind when using the rules as you may get a duplicated device.

- **OS30A:** As a note, you  do not need a rules for the OS30A, as it is configured to use its serial in the launch files for the camera.

- **ESP32:** As there are two ESP32 and there is a high probability you bought the two ESP32 from the same batch, they may end with the same attributes. That is the reason, you should prefer a environmental attribute like the **ID_PATH** to distinguis both devices.

    Also make sure that the ESP32 of the first floor is the one with the name **/dev/ttyESP32_1** and the one in the fourth floor is the **/dev/ttyESP32_2**, as the PlatformIO and µ-ROS programs will use this ports by default.

    For more inofrmation on the ESP32 and the udev rules, check [orion_base](/orion_base/README.md).

## 🚀 Launch files

### bringup.launch.py

The [bringup.launch.py](/orion_bringup/launch/bringup.launch.py) file will take charge of initializing the robot on the Raspberry PI, so it can load the description, camera drivers, controllers and µ-ROS nodes so the robot have full functionalities. It can be added to a startup service to be ready.

~~~bash
# Basic usage:
# ros2 launch orion_bringup bringup.launch.py
# Additional arguments:
#   camera : Can be 'astra_s', 'a010' or 'os30a'.
#   g_mov : Boolean (true/false) to use g_mov module when using 'a010' depth cam.
#   rasp : Whether to use 'rpi4' or 'rpi5', this will imply a change in the sound hardware.
#   servo : Boolean (true/false) to indicate if use servo arms
#   ros2_control : Boolean (true/false) to indicate usage of ros2_controllers
#   simplified : Boolean (true/false) to indicate if use the simplified URDF model
#   ctl_type: Control type can be 'micro-ros' or 'serial'.
#   motor : Motor rpms to select your motor params (for now, '100' or '1000')
ros2 launch orion_bringup bringup.launch.py camera:=a010
~~~

## 🚀 Setup of bringup on Startup

You can enable the robots bringup launch in order to start as the Raspberry Pi is being initialized, for this case, we will use services and actions for **systemd**.

### 📋 Prerequisites

1. **Ubuntu/Linux-based OS** installed on the Raspberry Pi (tested with Ubuntu 24.04).
2. **ROS2 Jazzy** installed and sourced correctly.
3. ROS2 workspace (e.g., `ros2_ws`) built and sourced.
4. Devices are connected this includes the two ESP32, your selected camera (OS30A, A010, ASTRA_S), the LIDAR LD19, speakers and microphone.
5. You have set up the **udev** rules.

### 📁 File Structure

Ensure you have the following elements:

1. You have a valid ROS 2 workspace:

    ~~~bash
    cd ~/ros2_ws/src
    ~~~

2. Add the content of the [startup_robot.sh](/orion_bringup/startup_robot.sh) to your local bin directory:

    ~~~bash
    sudo nano /usr/local/bin/startup_robot.sh
    ~~~

    Do not forget to check that the script works before proceeding:

    ~~~sh
    #!/bin/bash
    CAMERA_TYPE="a010"  # Options: a010, astra_s, os30a

    echo "Selected camera: $CAMERA_TYPE"

    echo "Waiting for internet connection..."
    while ! ping -c 1 8.8.8.8 &>/dev/null; do
        sleep 1
    done
    echo "Internet connection established."

    declare -a SYMLINK_DEVICES=("/dev/ttyESP32_1" "/dev/ttyESP32_2" "/dev/ttyLD19")

    for DEV in "${SYMLINK_DEVICES[@]}"; do
        echo "Waiting for $DEV..."
        while [ ! -e "$DEV" ]; do
            sleep 1
        done
        echo "$DEV found."
        
        REAL_DEV=$(readlink -f "$DEV")
        echo "Setting permissions for $REAL_DEV..."
        chmod 666 "$REAL_DEV"
    done

    if [ "$CAMERA_TYPE" == "a010" ]; then
        echo "Waiting for /dev/ttyA010..."
        while [ ! -e /dev/ttyA010 ]; do
            sleep 1
        done
        echo "/dev/ttyA010 is available."

    elif [ "$CAMERA_TYPE" == "astra_s" ]; then
        echo "Waiting for Astra S USB device..."
        while ! lsusb | grep -i "astra_s" &>/dev/null; do
            sleep 1
        done
        echo "Astra S USB device found."
    fi

    echo "Sourcing ROS2 and workspace..."
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash # Update with your workspace name
    # export ROS_DOMAIN_ID=16 # Only valid if not using µ-ROS

    echo "Launching ORION bringup..."

    if [ "$CAMERA_TYPE" == "a010" ] || [ "$CAMERA_TYPE" == "astra_s" ]; then
        ros2 launch orion_bringup bringup.launch.py camera:="$CAMERA_TYPE"
    else
        ros2 launch orion_bringup bringup.launch.py
    fi
    ~~~

3. Add the service [startup_robot.service](/orion_bringup/startup_robot.service) to the systemd directory:

    ~~~bash
    sudo nano /etc/systemd/system/startup_robot.service
    ~~~

    The content to consider is shown below, do not add comments to this file and do not forget to change the paths depending on your user.

    ~~~service
    [Unit]
    Description=Startup ROS2 Bringup Service
    After=network-online.target
    Wants=network-online.target

    [Service]
    ExecStart=/usr/local/bin/startup_robot.sh
    User=orion
    Environment=HOME=/home/orion
    WorkingDirectory=/home/orion
    Restart=on-failure
    StandardOutput=journal
    StandardError=journal

    [Install]
    WantedBy=multi-user.target
    ~~~

### 🛠️ Step-by-Step Setup

1. Give permission to the startup script:

    ~~~bash
    sudo chmod +x /usr/local/bin/startup_robot.sh
    ~~~

    Do not forget to update the script if your user, camera type, or device names change.

2. Reload **systemd** and Enable the startup robot service

    ~~~bash
    sudo systemctl daemon-reexec
    sudo systemctl daemon-reload
    sudo systemctl enable startup_robot.service
    ~~~

    To start it immediately without rebooting:

    ~~~bash
    sudo systemctl start startup_robot.service
    ~~~

3. Verify the Service Status (do this after launching it manually and at the robot startup)

    ~~~bash
    systemctl status startup_robot.service
    ~~~

4. View Service Logs and check everything is ok:

    ~~~bash
    journalctl -u startup_robot.service
    ~~~

    For real-time logs:

    ~~~bash
    journalctl -u startup_robot.service -f
    ~~~

---

### 🧪 Testing the Setup

1. **Reboot** the Raspberry Pi:

    ~~~bash
    sudo reboot
    ~~~

2. Confirm the service starts correctly using:

    ~~~bash
    systemctl status startup_robot.service
    ~~~

    or by checking logs:

    ~~~bash
    journalctl -u startup_robot.service -b
    ~~~

3. Also, open a terminal inside the Pi, and review the topics:

    ~~~bash
    ros2 topic list
    # It should display the description, control and µ-ROS topics
    ~~~

## ⚠️ Troubleshooting

### Startup service not loading

This may occur for the next reasons:

- The robot isn't connected to the internet, make sure to have a valid network for the ROS 2 communication

- One (or several) of the USB devices (ESP32, cameras or LIDAR) isn't connected and the startup service is waiting for the connection.

- The USB devices permissions didn't work, check them with:

    ~~~bash
    ls -al /dev | grep USB
    ~~~

    Use chmod on the device that no longer has read/write permissions.

    ~~~bash
    sudo chmod 777 /dev/ttyLD19
    sudo chmod 666 /dev/ttyUSB0
    ~~~

    Also, do not forget to add the user to the **dialout**.

### Robot is not moving

This may be caused if the µ-ROS nodes didn't activate on time or the connection with the agend wasn't possible. For this, just reboot the ESP32 by taking out the respective walls.

For more information check [orion_base](/orion_base/README.md)
