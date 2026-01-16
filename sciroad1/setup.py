from setuptools import find_packages, setup

package_name = 'sciroad1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/sciroad1']),
        ('share/sciroad1', ['package.xml']),
        ('share/sciroad1/launch', ['launch/gimbal.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yj',
    maintainer_email='yjyj37623@gmail.com',
    description='Turntable control and related nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gimbal_control_node = sciroad1.turntable_control_node:main',
        ],
    },
)

