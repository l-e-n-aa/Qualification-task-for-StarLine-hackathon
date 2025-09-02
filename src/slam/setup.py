from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py'))),
        (os.path.join('share', package_name, 'tb_office_v02'), glob(os.path.join('tb_office_v02', '*.mcap'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name), glob('*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lena',
    maintainer_email='lena.wty@mail.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pointcloud_to_laserscan = slam.pointcloud_to_laserscan:main', 'marker_publisher = slam.marker_publisher:main',
        ],
    },
)
