import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'amr_test'

    # RSP launch
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 
                'launch', 
                'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo launch with a custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'), 
                'launch', 
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory(package_name), 
                'worlds', 
                'amr_test.world'
            )
        }.items()
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot'
        ],
        output='screen'
    )

    # RViz2 launch with a delay
    rviz2 = TimerAction(
        period=2.5,  # Delay (in seconds) before starting RViz2
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d', os.path.join(
                        get_package_share_directory(package_name),
                        'config',
                        'lidar.rviz'
                    )
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz2,  # RViz2 starts after a delay
    ])
