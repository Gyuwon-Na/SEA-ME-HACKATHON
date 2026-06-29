import os
from glob import glob
from setuptools import setup

package_name = 'bisa'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='topst',
    maintainer_email='topst@todo.todo',
    description='BISA launch package for D-Racer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)