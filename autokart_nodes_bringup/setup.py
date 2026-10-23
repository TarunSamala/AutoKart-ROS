from setuptools import find_packages, setup

package_name = 'autokart_nodes_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/disarmed_sim.launch.py',
            'launch/replay_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Launch-only full-graph integration for the autokart_ros_nodes library.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_armed_bridge = autokart_nodes_bringup.safety_armed_bridge:main',
            'mapping_path_bridge = autokart_nodes_bringup.mapping_path_bridge:main',
            'replay_publisher = autokart_nodes_bringup.replay_publisher:main',
        ],
    },
)
