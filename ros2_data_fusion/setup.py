from setuptools import find_packages, setup

package_name = 'ros2_data_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ros2_data_fusion']),
        ('share/ros2_data_fusion', ['package.xml']),
        ('share/ros2_data_fusion/launch', ['launch/data_fusion.launch.py']),
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
        'data_fusion_node = ros2_data_fusion.data_fusion_node:main',
    ],
},

)
