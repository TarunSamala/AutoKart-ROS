from setuptools import find_packages, setup

package_name = 'autokart_perception_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_adapter.launch.py']),
        ('share/' + package_name + '/config', ['config/synthetic.yaml', 'config/topics_live.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Standalone YOLO26s + RGB/depth-fusion cone perception node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_adapter_node = autokart_perception_adapter.node:main',
        ],
    },
)
