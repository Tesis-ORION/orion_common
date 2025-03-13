import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)

def generate_robot_description(context):
    # -------------------------- Paths --------------------------------------
    pkg_gmov = get_package_share_directory('g_mov_description')
    pkg_description = get_package_share_directory('orion_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'orion.urdf.xacro')

    # --------------------------- Configurations -----------------------------
    camera = get_argument(context, "camera")
    servo = get_argument(context, "servo")
    g_mov = get_argument(context, "g_mov")
    rasp = get_argument(context, "rasp")
    gazebo = get_argument(context, "gazebo")

    mappings = {
        'camera': camera,
        'servo': servo,
        'g_mov': g_mov,
        'rasp': rasp,
        'gazebo': gazebo
    }

    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    
    # Passing absolute path to the robot description due to Gazebo issues finding andino_description pkg path.
    robot_desc = robot_desc.replace(
        'package://orion_description/', f'file://{pkg_description}/'
    )
    robot_desc = robot_desc.replace(
        'package://g_mov_description/', f'file://{pkg_gmov}/'
    )

    # -------------------------- Nodes ----------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'rate': 200,
        }]
    )

    return [rsp_node]


def generate_launch_description():
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
    

    return LaunchDescription([
        camera_arg,
        use_servo_arg,
        use_g_mov_arg,
        rasp_arg,
        gazebo_arg,
        OpaqueFunction(function=generate_robot_description),
    ])