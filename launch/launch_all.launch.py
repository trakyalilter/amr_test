import os
import math
from tf_transformations import quaternion_from_euler
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

    # Convert RPY (roll, pitch, yaw) to Quaternion for static transforms
    roll, pitch, yaw = -math.pi / 2, 0, 0
    quaternion_left = quaternion_from_euler(roll, pitch, yaw)
    static_tf_front_left_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.20', '0.175', '0',              # XYZ position
            str(quaternion_left[0]),           # Quaternion x
            str(quaternion_left[1]),           # Quaternion y
            str(quaternion_left[2]),           # Quaternion z
            str(quaternion_left[3]),           # Quaternion w
            'base_link', 'front_left_wheel'    # Parent and child frame
        ]
    )

    roll = math.pi / 2
    quaternion_right = quaternion_from_euler(roll, pitch, yaw)
    static_tf_front_right_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.20', '-0.175', '0',             # XYZ position
            str(quaternion_right[0]),          # Quaternion x
            str(quaternion_right[1]),          # Quaternion y
            str(quaternion_right[2]),          # Quaternion z
            str(quaternion_right[3]),          # Quaternion w
            'base_link', 'front_right_wheel'   # Parent and child frame
        ]
    )

    # SLAM Toolbox launch with an 8-second delay
    slam_toolbox = TimerAction(
        period=8.0,  # 8 seconds (after RViz2 starts)
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    )
                ]),
                launch_arguments={
                    'params_file': os.path.join(
                        get_package_share_directory(package_name),
                        'config',
                        'mapper_params_online_async.yaml'
                    ),
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )

    # Twist Mux launch with a 10-second delay
    twist_mux = TimerAction(
        period=10.0,  # 10 seconds (after SLAM Toolbox starts)
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('twist_mux'),
                        'launch',
                        'twist_mux_launch.py'
                    )
                ])
            )
        ]
    )

    # Nav2 Bringup launch with a 12-second delay
    nav2 = TimerAction(
        period=12.0,  # 12 seconds (after Twist Mux starts)
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    )
                ]),
                launch_arguments={
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        rsp,                                # Start RSP immediately
        static_tf_front_left_wheel,        # Publish transform for front left wheel
        static_tf_front_right_wheel,       # Publish transform for front right wheel
        gazebo,                             # Start Gazebo after 2 seconds
        spawn_entity,                       # Spawn robot after 4 seconds
        rviz2,                              # Start RViz2 after 6 seconds
        slam_toolbox,                       # Start SLAM Toolbox after 8 seconds
        twist_mux,                          # Start Twist Mux after 10 seconds
        nav2                                # Start Nav2 Bringup after 12 seconds
    ])
