import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'amr_test'

    # RSP launch (starts immediately)
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

    # Gazebo launch with a 2-second delay
    gazebo = TimerAction(
        period=2.0,  # 2-second delay
        actions=[
            IncludeLaunchDescription(
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
        ]
    )

    # Spawn the robot entity with a 2-second delay
    spawn_entity = TimerAction(
        period=4.0,  # 4 seconds (after Gazebo starts)
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'my_bot'
                ],
                output='screen'
            )
        ]
    )

    # RViz2 launch with a 2-second delay
    rviz2 = TimerAction(
        period=6.0,  # 6 seconds (after spawn_entity starts)
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
        rsp,        # Start RSP immediately
        gazebo,     # Start Gazebo after 2 seconds
        spawn_entity,  # Start spawn_entity after 4 seconds
        rviz2       # Start RViz2 after 6 seconds
    ])
