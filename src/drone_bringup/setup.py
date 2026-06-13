from setuptools import find_packages, setup

package_name = 'drone_bringup'

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
    description='Control command generation node for the drone vision system',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_node = drone_bringup.control_node:main',
        ],
    },
)