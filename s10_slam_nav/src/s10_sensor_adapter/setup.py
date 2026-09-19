from setuptools import find_packages, setup


package_name = 's10_sensor_adapter'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/imu_adapter.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='S10 Team',
    maintainer_email='team@example.com',
    description='S10 Airy IMU coordinate-frame adapter.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'imu_adapter = s10_sensor_adapter.imu_adapter:main',
        ],
    },
)
