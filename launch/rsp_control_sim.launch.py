import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

# Lidar launch command 
"""
ros2 run rplidar_ros rplidar_composition --ros-args --remap serial_port:=/dev/ttyUSB0 --param frame_id:=rplidar_sensor_v1_1_1 --param angle_compensate:=true --param scan_mode:=Standard --param serial_baudrate:=115200

"""

#Camera launch command
"""
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_link_optical

"""

# motor demo commands
"""
dev machine:
ros2 run serial_motor_demo gui

rpi:
ros2 run serial_motor_demo driver --ros-args -p serial_por:=/dev/ttyUSB0 -p baud_rate:=57600 -p loop_rate:=30 -p encoder_cpr:=2800
"""

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('AROBO7'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    
    
#-- new xacro things --
# Define the paths to the included files
    
    materials_file = os.path.join(pkg_path, 'description', 'materials.xacro')
    trans_file = os.path.join(pkg_path, 'description', 'AROBO7.trans')
    gazebo_file = os.path.join(pkg_path, 'description', 'AROBO7.gazebo')
    
 # Process the xacro file with arguments
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'materials_file': materials_file,
            'trans_file': trans_file,
            'gazebo_file': gazebo_file,
            'use_ros2_control': 'true',
            'use_sim_mode': 'true'
        }
    ).toxml()

    # robot_description_config = Command([
    #     'xacro', xacro_file,
    #     'materials_file:=' + materials_file,
    #     'trans_file:=' + trans_file,
    #     'gazebo_file:=' + gazebo_file,
    #     'use_ros2_control:=' + use_ros2_control
    # ])

#-- end xacro things --

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
            
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher
    ])
