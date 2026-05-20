from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'bno085_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danitns',
    maintainer_email='danitns@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_data_publisher = bno085_ros2_driver.imu_data_publisher:main',
        ],
    },
)
