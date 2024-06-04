import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

#TO RUN THIS SCRIPT: ros2 launch AROBO7 launch_sim.launch.py


#Run keyboard control command:
#ros2 run teleop_twist_keyboard teleop_twist_keyboard
def generate_launch_description():

    package_name = 'AROBO7'

    #Include the robot state publisher (rsp)
    #ros2 launch AROBO7 rsp.launch.py use_sim_time:=true
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    #include the gazebo launch file
    #ros2 launch gazebo_ros gazebo.launch.py extra_gazebo_args:="--verbose"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
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
    
    #Launch them all!
    return LaunchDescription([
        rsp, gazebo, spawn_entity # teleop
    ])