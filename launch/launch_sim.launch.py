import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# TO RUN THIS SCRIPT: ros2 launch AROBO7 launch_sim.launch.py
# with maze.world: ros2 launch AROBO7 launch_sim.launch.py world:=./src/AROBO7/worlds/maze.world

# Run keyboard control command: ros2 run teleop_twist_keyboard teleop_twist_keyboard
# with remapped topic: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

def generate_launch_description():

    package_name = 'AROBO7'

    #Include the robot state publisher (rsp)
    #ros2 launch AROBO7 rsp.launch.py use_sim_time:=true
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    #include the gazebo launch file
    #ros2 launch gazebo_ros gazebo.launch.py extra_gazebo_args:="--verbose"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'extra+gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )   

  

    #run teleop twist for controls
    #teleop = Node(
    #    package='teleop_twist_keyboard',
    #    executable='teleop_twist_keyboard',
    #    name='teleop_twist_keyboard',
    #    output='screen'
    #)

    #Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'AROBO7',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0'],
                   output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    #Launch them all!
    return LaunchDescription([
        rsp, gazebo, spawn_entity, diff_drive_spawner, joint_broad_spawner
    ])