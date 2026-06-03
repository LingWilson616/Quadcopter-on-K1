from setuptools import setup, find_packages

package_name = 'drone_vision'

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
    description='Camera capture and image processing node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = drone_vision.camera_node:main',
        ],
    },
)
