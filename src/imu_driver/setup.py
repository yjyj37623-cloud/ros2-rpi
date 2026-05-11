from setuptools import find_packages, setup

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/imu_driver']),
    ('share/imu_driver', ['package.xml']),
    ('share/imu_driver/launch', ['launch/imu.launch.py']),  # <-- 加上这行
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yj',
    maintainer_email='yj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = imu_driver.imu_node:main',
        ],
    },
)

