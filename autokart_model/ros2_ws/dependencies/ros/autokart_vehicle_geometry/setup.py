from setuptools import setup

package_name = "autokart_vehicle_geometry"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/autokart_vehicle_geometry"]),
        ("share/autokart_vehicle_geometry", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="AutoKart",
    maintainer_email="simulation@example.invalid",
    description="Single source of truth for kart/camera/bumper geometry (no ROS dependency). Loads "
                "kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "print_vehicle_geometry = autokart_vehicle_geometry.geometry:main",
    ]},
)
