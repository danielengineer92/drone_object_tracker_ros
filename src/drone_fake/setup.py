from setuptools import find_packages, setup

package_name = 'drone_fake'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Drone Vision Team',
    maintainer_email='drone@roche.com',
    description='Fake/simulated nodes for testing the drone vision system without hardware',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_camera_node = drone_fake.fake_camera_node:main',
            'fake_detection_node = drone_fake.fake_detection_node:main',
            'fake_telemetry_node = drone_fake.fake_telemetry_node:main',
        ],
    },
)