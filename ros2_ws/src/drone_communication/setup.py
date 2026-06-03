from setuptools import setup, find_packages

package_name = 'drone_communication'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bianbu',
    maintainer_email='bianbu@spacemit.com',
    description='UART/MAVLink communication with ArduPilot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mavlink_node = drone_communication.mavlink_node:main',
        ],
    },
)
