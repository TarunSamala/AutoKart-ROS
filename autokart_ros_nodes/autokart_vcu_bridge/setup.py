from setuptools import find_packages, setup

package_name = 'autokart_vcu_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vcu_bridge.launch.py']),
        ('share/' + package_name + '/config',
         ['config/vcu_bridge_synthetic.yaml', 'config/vcu_bridge_live.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='VCU USB/serial bridge ROS 2 node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vcu_bridge_node = autokart_vcu_bridge.node:main',
        ],
    },
)
