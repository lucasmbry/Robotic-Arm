import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package and URDF file
    package_name = "six_dof"
    xacro_file = "URDF.xacro"

    # Path to URDF file
    xacro_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        xacro_file
    )

    # Path to RViz config
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'urdf_config.rviz'
    )

    # Convert xacro to urdf
    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    # This is what is run
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])