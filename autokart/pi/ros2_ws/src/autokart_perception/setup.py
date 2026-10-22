from setuptools import find_packages, setup

package_name = "autokart_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autokart",
    maintainer_email="harrishkesavan@gmail.com",
    description="autokart_perception",
    license="Proprietary",
    entry_points={"console_scripts": ["autokart_perception_node = autokart_perception.node:main"]},
)
