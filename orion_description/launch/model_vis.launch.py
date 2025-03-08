import os

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
    use_gui = LaunchConfiguration('use_gui')
    camera = LaunchConfiguration('camera')
    servo = LaunchConfiguration('servo')
    g_mov = LaunchConfiguration('g_mov')
    rasp = LaunchConfiguration('rasp')

    # -------------------------- Launch arguments -----------------------------
    gui_arg = DeclareLaunchArgument(
        "use_gui", 
        default_value="true", 
        description="Use joint_state_publisher_gui"
    )

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


    # -------------------------- Nodes ----------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', xacro_file, 
                ' camera:=', camera,
                ' servo:=', servo,
                ' g_mov:=', g_mov,
                ' rasp:=', rasp,
            ])
        }]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_config_file = os.path.join(pkg_description, 'rviz', 'model_viz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        camera_arg,
        use_servo_arg,
        use_g_mov_arg,
        rasp_arg,
        gui_arg,
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])
