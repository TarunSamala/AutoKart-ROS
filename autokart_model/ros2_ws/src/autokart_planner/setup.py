from setuptools import find_packages, setup

package_name = 'autokart_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py']),
        ('share/' + package_name + '/config', [
            'config/planner_builtin.yaml',
            'config/planner_reused.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Standalone trajectory/curvature/collision/speed-profile/stop-envelope planner ROS 2 node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node = autokart_planner.node:main',
        ],
    },
)
