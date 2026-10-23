from setuptools import find_packages, setup

package_name = 'autokart_hp60c'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hp60c.launch.py']),
        ('share/' + package_name + '/config', ['config/hp60c_synthetic.yaml', 'config/hp60c_live.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoKart',
    maintainer_email='harrishkesavan@gmail.com',
    description='Sole HP60C camera SDK owner ROS 2 node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hp60c_node = autokart_hp60c.node:main',
        ],
    },
)
