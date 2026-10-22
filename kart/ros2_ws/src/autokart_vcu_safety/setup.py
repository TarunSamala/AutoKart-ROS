from setuptools import setup

package_name = "autokart_vcu_safety"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/autokart_vcu_safety"]),
        ("share/autokart_vcu_safety", ["package.xml", "config/vcu_safety.yaml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="AutoKart",
    maintainer_email="simulation@example.invalid",
    description="VCU arming safety supervisor: the only node that may request ARM of autokart_vehicle_bridge. "
                "Distinct from autokart_safety_bridge's diagnostic-only front obstacle envelope monitor.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "vcu_safety_node = autokart_vcu_safety.node:main",
    ]},
)
