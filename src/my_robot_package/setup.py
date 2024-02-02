# setup.py
from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_original_nodes.py', 'launch/launch_new_nodes.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A brief description of the package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = my_robot_package.node1:main',
            'video_subscriber = my_robot_package.node2:main',
            'servo_control = my_robot_package.node3:main',
            'video_subscriber_v2 = my_robot_package.node2_v2:main',
            'servo_control_v2 = my_robot_package.node3_v2:main',
        ],
    },
)