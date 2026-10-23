from setuptools import find_packages, setup

package_name = 'autokart_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odometry.launch.py']),
        ('share/' + package_name + '/config',
         ['config/odometry_synthetic.yaml', 'config/odometry_live.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Wheel-encoder + observed-steering odometry ROS 2 node (no IMU fusion).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = autokart_odometry.node:main',
        ],
    },
)
