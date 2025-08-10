from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess # чтобы воспроизвести tb_office_v02_0.mcap
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'slam'
    package_share = get_package_share_directory(package_name) # возвращает абсолютный путь к директории share
    bag_file = os.path.join(package_share, 'bags', 'tb_office_v02_0.mcap') # собирает полный путь к bag файлу
    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file, '--clock'],
            output='screen',
            
        )
        ,
        Node(
            package='slam',
            executable='pointcloud_to_laserscan',
            name='pc2_to_laserscan',
            output='screen',
            parameters=[
                {'min_height': -0.1},
                {'max_height': 0.1},
                {'min_range': 0.1},
                {'max_range': 10.0},
                {'angle_min': -3.14},   
                {'angle_max': 3.14},    
                {'alpha_increment': 0.0087},
                {'scan_frame': 'livox'},
                {'use_sim_time': True}
            ]
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # используем время из bag-файла
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }],
            remappings=[
                ('/scan', '/scan')  # подписываемся на топик со сканами
            ]
        ),
    ])
