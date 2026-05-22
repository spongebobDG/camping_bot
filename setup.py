import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camping_bot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spbdg',
    maintainer_email='eorjs135795@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_publisher = camping_bot_bridge.esp32_odom_publisher:main',
            'lidar_bridge = camping_bot_bridge.lidar_udp_bridge:main',
        ],
    },
)
