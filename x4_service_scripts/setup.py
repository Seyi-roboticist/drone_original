from setuptools import find_packages, setup

package_name = 'x4_service_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lavinia Kong',
    maintainer_email='TODO@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "dds_takeoff = x4_service_scripts.dds_takeoff:main",
            "dds_waypoint = x4_service_scripts.dds_waypoint:main",
            "mavros_takeoff = x4_service_scripts.mavros_takeoff:main",
            "mavros_waypoint = x4_service_scripts.mavros_waypoint:main",
            "mavros_landing = x4_service_scripts.mavros_landing:main",
            "mavros_loiter_mode = x4_service_scripts.mavros_loiter_mode:main",
            "mavros_takeoff_RTL = x4_service_scripts.mavros_takeoff_RTL:main",
        ],
    },
)
