from launch_ros.substitutions import FindPackageShare

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node

import math

use_sim_time = LaunchConfiguration('use_sim_time', default='false')
resolution = LaunchConfiguration('resolution', default='0.05')
publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

PI = math.pi

def generate_launch_description():
    #rviz_file = os.path.join(get_package_share_directory('merge_map'), 'config', 'merge_map.rviz')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros2_turtlebot_controller'),
                    'launch',
                    'Lima_cartographer.launch.py'
                ])
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros2_turtlebot_controller'),
                    'launch',
                    'Alpha_cartographer.launch.py',
                ])
            ),
        ),
    
        Node(
            package='ros2_turtlebot_controller',
            namespace='Lima',
            executable='cmd_publisher_node',
            output='screen',
            parameters=[{'goal1': [1, 0, PI, 0, 0, 0]},
                        #{'goal2': [0, 0, 0]},
                        {'robot_name': 'Lima'},                        
                        ]
        ),

        Node(
            package='ros2_turtlebot_controller',
            namespace='Alpha',
            executable='cmd_publisher_node',
            output='screen',
            parameters=[{'goal1': [1, 2, PI, 0, 2, 0]},
                        #{'goal2': [1, 0, 0]},
                        {'robot_name': 'Alpha'},  
                        ]
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_file],
        #     parameters=[{'use_sim_time': True}]
        # ),
        
        Node(
            package='merge_map',
            executable='merge_map',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ("/map1", "/Lima/map"),
                ("/map2", "/Alpha/map"),
                ],
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'Lima_map']
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '2', '0', '0', '0', '0', 'map', 'Alpha_map']
        ),

    ])
