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
        # Top-level checkpoint files only (glob('*') would also match the
        # best_ncnn_model/ dir, which data_files cannot copy as a unit).
        (os.path.join('share', package_name, 'checkpoints'),
         [p for p in glob('checkpoints/*') if os.path.isfile(p)]),
        # NCNN export is a directory of files (param/bin/metadata) — install
        # each file into checkpoints/best_ncnn_model/ so YOLO() can load it.
        (os.path.join('share', package_name, 'checkpoints', 'best_ncnn_model'),
         [p for p in glob('checkpoints/best_ncnn_model/*') if os.path.isfile(p)]),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='topst',
    maintainer_email='topst@todo.todo',
    description='BISA launch package for D-Racer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bisa_detector_node = bisa.detector_node:main',
            'viz_node = bisa.viz_node:main',
            'param_gui_node = bisa.param_gui_node:main',
            'traffic_light_tuner = bisa.traffic_light:main',
            'dash_line_tuner = bisa.dash_line_tuner:main',
        ],
    },
)
