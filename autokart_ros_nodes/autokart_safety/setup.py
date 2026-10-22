from setuptools import find_packages, setup

package_name = 'autokart_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/safety.launch.py']),
        ('share/' + package_name + '/config', [
            'config/safety_synthetic.yaml', 'config/safety_live.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Software-only vehicle arming safety authority ROS 2 node (not the physical E-stop).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = autokart_safety.node:main',
        ],
    },
)
