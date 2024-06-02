import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

#TO RUN THIS SCRIPT: ros2 launch AROBO7 launch_rviz.launch.py

def generate_launch_description():

    package_name = 'AROBO7'

    #Include the robot state publisher (rsp)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Run the joint_state_publisher_gui node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Run the RViz node
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'view_bot.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    #Launch them all!
    return LaunchDescription([
        rsp, joint_state_publisher_gui, rviz_node
    ])