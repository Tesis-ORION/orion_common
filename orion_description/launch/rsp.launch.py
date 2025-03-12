import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Define paths
    pkg_description = get_package_share_directory('orion_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'orion.urdf.xacro')

    # --------------------------- Configurations -----------------------------
    camera = LaunchConfiguration('camera')
    servo = LaunchConfiguration('servo')
    g_mov = LaunchConfiguration('g_mov')
    rasp = LaunchConfiguration('rasp')
    gazebo = LaunchConfiguration('gazebo')

    # -------------------------- Launch arguments -----------------------------
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='a010',
        description="Choose a cam for the robot (os30a, astra_s, a010)"
    )

    use_servo_arg = DeclareLaunchArgument(
        'servo',
        default_value='true',
        description="Boolean to include or not the servos"
    )

    use_g_mov_arg = DeclareLaunchArgument(
        'g_mov',
        default_value='true',
        description="When using camera a010, whether to include or not G Mov"
    )
    
    rasp_arg = DeclareLaunchArgument(
        'rasp',
        default_value='rpi5',
        description="Select 4 for Raspberry Pi 4B, or 5 for Raspberry Pi 5"
    )

    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='true',
        description="True for using gazebo tags, false otherwise"
    )

    # -------------------------- Additional processes ------------------------
    mappings = {
        'camera':'a010',
        'servo':'true',
        'g_mov':'false',
        'rasp':'rpi5',
        'gazebo':'true'
        }
    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    # Passing absolute path to the robot description due to Gazebo issues finding andino_description pkg path.
    robot_desc = robot_desc.replace(
        'package://orion_description/', f'file://{pkg_description}/'
    )

    # -------------------------- Nodes ----------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
        }]
    )

    return LaunchDescription([
        camera_arg,
        use_servo_arg,
        use_g_mov_arg,
        rasp_arg,
        gazebo_arg,
        rsp_node,
    ])