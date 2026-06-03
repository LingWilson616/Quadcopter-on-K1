from setuptools import setup, find_packages

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bianbu',
    maintainer_email='bianbu@spacemit.com',
    description='Launch files for K1 drone project',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
