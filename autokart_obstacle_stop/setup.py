from setuptools import find_packages, setup

package_name = "autokart_obstacle_stop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/obstacle_stop.launch.py"]),
        ("share/" + package_name + "/config", ["config/obstacle_stop.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AutoKart",
    maintainer_email="harrishkesavan@gmail.com",
    description="Fail-closed forward-cone stop guard for AutoKart ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_stop_node = autokart_obstacle_stop.node:main",
        ],
    },
)

