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
    rviz_file = os.path.join(get_package_share_directory('merge_map'), 'config', 'merge_map.rviz')
    return LaunchDescription([
        # Launch from this package
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
            remappings=[
                ('/cmd_vel','/Lima/cmd_vel'),
                ('/imu_link', '/Lima/Lima_imu_link'),
                #('/map', '/Lima/map'),
            ]
        ),
        Node(
            package='ros2_turtlebot_controller',
            namespace='Alpha',
            executable='cmd_publisher_node',
            output='screen',
            remappings=[
                ('/cmd_vel','/Alpha/cmd_vel'),
                ('imu_link', 'Alpha_imu_link'),
                #('/map', '/Alpha/map'),
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}]
        ),
        
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
            arguments=['1', '0', '0', '0', '0', '0', 'map', 'Alpha_map']
        ),

    ])
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map Lima_map

# def generate_launch_description():
#     ld = LaunchDescription()

#     # LiDAR 데이터 통합을 위한 mux 노드
#     mux_node = Node(
#         package='topic_tools',
#         executable='mux',
#         name='scan_mux',
#         output='screen',
#         remappings=[
#             ('/input', '/Lima/scan'),   # 첫 번째 로봇의 LiDAR 데이터
#             ('/input_2', '/Alpha/scan'), # 두 번째 로봇의 LiDAR 데이터
#             ('/output', '/scan')         # Cartographer에서 사용할 스캔 데이터
#         ]
#     )
#     ld.add_action(mux_node)

#     cartographer_node = Node(
#         package='cartographer_ros',
#         executable='cartographer_node',
#         name='cartographer',
#         output='screen',
#         parameters=[{
#             'use_sim_time': use_sim_time,
#             'map_filename': '/home/map.pgm',  
#         }],
#         remappings=[
#             ('/scan', '/scan'),  # 통합된 스캔 데이터
#         ]
#     )
#     ld.add_action(cartographer_node)


#     controller_node_Lima = Node(
#         package='ros2_turtlebot_controller',
#         namespace='Lima',
#         executable='cmd_publisher_node',
#         output='screen',
#         parameters=[{'goal': [1, 1, PI]}],
#         remappings=[
#             ('/imu_link', '/Lima/Lima_imu_link'),
#         ]
#     )
#     ld.add_action(controller_node_Lima)

#     controller_node_Alpha = Node(
#         package='ros2_turtlebot_controller',
#         namespace='Alpha',
#         executable='cmd_publisher_node',
#         output='screen',
#         parameters=[{'goal': [0, 0, 0]}],
#         remappings=[
#             ('/imu_link', '/Alpha/Alpha_imu_link'),
#         ]
#     )
#     ld.add_action(controller_node_Alpha)

#     return ld
