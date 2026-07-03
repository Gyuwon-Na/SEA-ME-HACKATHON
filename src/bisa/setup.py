import os
from glob import glob
from setuptools import setup

package_name = 'bisa'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={package_name: 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'checkpoints'), glob('checkpoints/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='topst',
    maintainer_email='topst@todo.todo',
    description='BISA launch package for D-Racer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bisa_autonomous_node = bisa.autonomous_driving_node:main',
            'viz_node = bisa.viz_node:main',
            'param_gui_node = bisa.param_gui_node:main',
            'power_gui_node = bisa.power_gui_node:main',
        ],
    },
)
