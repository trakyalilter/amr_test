import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name='amr_test'
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                    'world':os.path.join(get_package_share_directory(package_name),
                    'worlds',
                    'amr_test.world')}.items()
             )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    rviz2 = Node(package='rviz2', executable='rviz2',
                 arguments=['-d',os.path.join(get_package_share_directory(package_name),'config','lidar.rviz')],output='screen')
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz2
    ])
