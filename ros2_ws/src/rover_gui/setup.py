from setuptools import find_packages, setup

package_name = 'rover_gui'

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
    description='PyQt operator GUI for monitoring and commanding the rover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = rover_gui.gui_node:main',
        ],
    },
)
