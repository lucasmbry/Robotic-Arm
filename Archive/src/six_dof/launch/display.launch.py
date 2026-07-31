import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Get the path to the package
    pkg_share = get_package_share_directory('six_dof')
    
    # 2. Get the paths to the URDF/Xacro and the RViz config
    xacro_file = os.path.join(pkg_share, 'urdf', 'URDF.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # 3. Tell ROS 2 to compile the Xacro file into a standard URDF string
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 4. Create the Robot State Publisher Node (The Architect)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 5. Create the Joint State Publisher GUI Node (The Sliders)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # 6. Create the RViz Node (The Viewer)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Launch them all together!
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])