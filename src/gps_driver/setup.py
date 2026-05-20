from setuptools import setup, find_packages

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
     data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gps_driver']),
        ('share/gps_driver', ['package.xml']),
        ('share/gps_driver/launch', ['launch/gps.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yj',
    maintainer_email='yj@todo.todo',
    description='GPS parsing and ROS2 publishing package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = gps_driver.gps_node:main',
        ],
    },
)

