from setuptools import find_packages, setup

package_name = 'ros2_serial_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ros2_serial_bridge']),
        ('share/ros2_serial_bridge', ['package.xml']),
        ('share/ros2_serial_bridge/launch', ['launch/serial_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yj',
    maintainer_email='yjyj37623@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'serial_bridge = ros2_serial_bridge.ros2_serial_bridge:main',
    ],
},

)
