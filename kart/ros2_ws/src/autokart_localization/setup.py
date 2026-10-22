from setuptools import setup

package_name = "autokart_localization"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/autokart_localization"]),
        ("share/autokart_localization", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="AutoKart",
    maintainer_email="simulation@example.invalid",
    description="Wheel-encoder odometry (axle speed + observed steering, bicycle model). "
                "No IMU in this configuration; yaw is a model estimate, not a measurement.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "localization_node = autokart_localization.node:main",
    ]},
)
