from setuptools import setup, find_packages

package_name = 'drone_voice'

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
    description='Voice interaction node — VAD+ASR+LLM+TTS pipeline',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'voice_node = drone_voice.voice_node:main',
        ],
    },
)
