#!/bin/bash

# === 1. Camera Selection (hardcoded or can be set externally) ===
CAMERA_TYPE="a010"  # Options: a010, astra_s, os30a

echo "Selected camera: $CAMERA_TYPE"

# === 2. Wait for Internet Connection ===
echo "Waiting for internet connection..."
while ! ping -c 1 8.8.8.8 &>/dev/null; do
    sleep 1
done
echo "Internet connection established."

# === 3. Ensure Symlinked TTY Devices Exist and Set Permissions ===
declare -a SYMLINK_DEVICES=("/dev/ttyESP32_1" "/dev/ttyESP32_2" "/dev/ttyLD19")

for DEV in "${SYMLINK_DEVICES[@]}"; do
    echo "Waiting for $DEV..."
    while [ ! -e "$DEV" ]; do
        sleep 1
    done
    echo "$DEV found."
    
    # Resolve real device and chmod it
    REAL_DEV=$(readlink -f "$DEV")
    echo "Setting permissions for $REAL_DEV..."
    chmod 666 "$REAL_DEV"
done

# === 4. Conditional Camera Device Checks ===

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

# === 5. Source ROS2 Environment ===
echo "Sourcing ROS2 and workspace..."
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
# export ROS_DOMAIN_ID=16

# === 6. Launch ROS2 with Appropriate Parameters ===
echo "Launching ORION bringup..."

if [ "$CAMERA_TYPE" == "a010" ] || [ "$CAMERA_TYPE" == "astra_s" ]; then
    ros2 launch orion_bringup bringup.launch.py camera:="$CAMERA_TYPE"
else
    ros2 launch orion_bringup bringup.launch.py
fi
