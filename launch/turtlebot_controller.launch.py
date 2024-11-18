from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration

from launch_ros.actions import Node

import math

use_sim_time = LaunchConfiguration('use_sim_time', default='false')

PI = math.pi

def generate_launch_description():
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
                    'Alpha_cartographer.launch.py'
                ])
            ),
        ),
    
        # Node(
        #     package='ros2_turtlebot_controller',
        #     namespace='Lima',
        #     #executable='cmd_publisher',
        #     executable='cmd_publisher_node',
        #     output='screen',
        #     parameters=[{'goal': [1, 1, PI]}],
        #     remappings=[
        #         ('/cmd_vel','/Lima/cmd_vel'),
        #         #('/imu_link', '/Lima/Lima_imu_link'),
        #         #('/map', '/Lima/map'),
        #     ]
        # ),
        # Node(
        #     package='ros2_turtlebot_controller',
        #     namespace='Alpha',
        #     #executable='cmd_publisher',
        #     executable='cmd_publisher_node',
        #     output='screen',
        #     parameters=[{'goal': [1, 1, PI]}],
        #     remappings=[
        #         ('/cmd_vel','/Alpha/cmd_vel'),
        #         #('imu_link', 'Alpha_imu_link'),
        #         #('/map', '/Alpha/map'),
        #     ]
        # ),
    ])

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
