from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess # чтобы воспроизвести tb_office_v02_0.mcap


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link', # Система координат, в которую преобразуются данные
                'transform_tolerance': 0.01,
                'min_height': -0.5, 
                'max_height': 0.5,
                'angle_min': -3.14,  
                'angle_max': 3.14,   
                'range_min': 0.1,
                'range_max': 50.0,
            }],
            remappings=[
                ('cloud_in', '/livox/lidar'), # откуда читаем данные
                ('scan', '/scan') # куда отправляем данные
            ]
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/lena/python files/attempt/src/slam/bags/tb_office_v02_0.mcap', '--clock'],
            output='screen'
        )
    ])
