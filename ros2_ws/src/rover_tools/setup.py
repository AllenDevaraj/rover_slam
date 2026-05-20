from setuptools import find_packages, setup

package_name = 'rover_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Developer, calibration, and debug utilities (not part of the runtime stack).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_odom = rover_tools.fake_odom:main',
            'calibrate_velocity = rover_tools.calibrate_velocity:main',
            'debug_vel = rover_tools.debug_vel:main',
            'imu_calib = rover_tools.imu_calib:main',
            'desktop_stream = rover_tools.desktop_stream:main',
        ],
    },
)
