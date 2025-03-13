from setuptools import find_packages, setup

package_name = 'modbus_slave'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rune',
    maintainer_email='143501123+espenakk@users.noreply.github.com',
    description='A ROS2 Modbus TCP slave node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modbus_slave_node = modbus_slave.modbus_slave_node:main'
        ],
    },
)