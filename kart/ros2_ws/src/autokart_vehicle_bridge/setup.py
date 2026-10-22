from setuptools import setup

package_name = "autokart_vehicle_bridge"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/autokart_vehicle_bridge"]),
        ("share/autokart_vehicle_bridge", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="AutoKart",
    maintainer_email="simulation@example.invalid",
    description="Real USB-serial bridge to vcu_firmware/ -- single owner of the VCU command link. "
                "NO-GO: not run against hardware yet.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "vehicle_bridge = autokart_vehicle_bridge.node:main",
        "run_logger = autokart_vehicle_bridge.run_logger:main",
    ]},
)
