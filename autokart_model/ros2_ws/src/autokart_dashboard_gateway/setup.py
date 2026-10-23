from setuptools import find_packages, setup

package_name = 'autokart_dashboard_gateway'
module_name = 'autokart_dashboard_gateway_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[module_name, module_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dashboard_gateway.launch.py']),
        ('share/' + package_name + '/config', [
            'config/dashboard_gateway_synthetic.yaml', 'config/dashboard_gateway_live.yaml',
        ]),
    ],
    install_requires=['setuptools', 'aiohttp>=3.8'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Read-only Mission Control ROS-to-WebSocket/HTTP gateway ROS 2 node (standalone wrapper).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_gateway_node = autokart_dashboard_gateway_node.node:main',
        ],
    },
)
