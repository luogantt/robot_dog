from setuptools import find_packages, setup


package_name = 's10_route_nav'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='S10 Team',
    maintainer_email='team@example.com',
    description='S10 route navigation and safety nodes.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'route_follower = s10_route_nav.route_follower:main',
            'route_recorder = s10_route_nav.route_recorder:main',
            'obstacle_guard = s10_route_nav.obstacle_guard:main',
            'health_monitor = s10_route_nav.health_monitor:main',
            'command_mux = s10_route_nav.command_mux:main',
        ],
    },
)
