from setuptools import find_packages, setup

package_name = 'autokart_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
        ('share/' + package_name + '/config', ['config/mission_skidpad.yaml',
                                               'config/mission_braking_test.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Standalone mission/braking-FSM ROS 2 node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = autokart_mission.node:main',
        ],
    },
)
