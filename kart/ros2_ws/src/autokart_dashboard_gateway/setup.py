from setuptools import setup

package_name = 'autokart_dashboard_gateway'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'aiohttp>=3.8'],
    zip_safe=True,
    maintainer='AutoKart Sim Studio',
    maintainer_email='simulation@example.invalid',
    description='Read-only Mission Control gateway',
    license='Apache-2.0',
    entry_points={'console_scripts': ['dashboard_gateway = autokart_dashboard_gateway.gateway:main']},
)
