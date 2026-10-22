from setuptools import setup

package_name = "autokart_sim_bridge"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/autokart_sim_bridge"]),
        ("share/autokart_sim_bridge", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="AutoKart Sim Studio",
    maintainer_email="simulation@example.invalid",
    description="VCU-gated AutoKart ROS 2 bridge",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "vcu_bridge = autokart_sim_bridge.vcu_bridge:main",
        "gazebo_full_lap = autokart_sim_bridge.gazebo_full_lap:main",
        "gazebo_sync_probe = autokart_sim_bridge.gazebo_sync_probe:main",
    ]},
)
