from setuptools import find_packages, setup

package_name = 'autokart_system_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system_monitor.launch.py']),
        ('share/' + package_name + '/config', [
            'config/system_monitor_synthetic.yaml', 'config/system_monitor_live.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Health and startup-evidence reporting ROS 2 node (no safety authority).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor_node = autokart_system_monitor.node:main',
        ],
    },
)
