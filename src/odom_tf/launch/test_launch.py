from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess # чтобы воспроизвести tb_office_v02_0.mcap
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    package_share = get_package_share_directory('odom_tf') # возвращает абсолютный путь к директории share

    #rviz_config = os.path.join(launch_dir, 'RViz_config.rviz')
    rviz_config = os.path.join(package_share, 'rviz', 'RViz_config.rviz') # собирает полный путь к slam.rviz файлу
    mcap_config = os.path.join(package_share, 'bags', 'tb_office_v02_0.mcap')
    return LaunchDescription([
        Node(
            package='odom_tf',
            executable='PC2_tf2',
            name='PC2_tf2',
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', mcap_config, '--loop'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}]  # Измените на True если используете симуляцию
        )

    ])