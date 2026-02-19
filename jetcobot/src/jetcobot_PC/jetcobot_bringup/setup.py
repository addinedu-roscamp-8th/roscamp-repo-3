from setuptools import setup, find_packages
import os
from glob import glob

setup(
    name='jetcobot_bringup',
    version='0.0.0',
    packages=find_packages(where='src', include=['jetcobot_bringup*']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'jetcobot_bringup']),
        ('share/' + 'jetcobot_bringup', ['package.xml']),
        (os.path.join('share', 'jetcobot_bringup', 'config'), glob('config/*.yaml')),
        (os.path.join('share', 'jetcobot_bringup', 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seo',
    maintainer_email='seo@todo.com',
    description='Launch files and basic nodes for starting jetcobot robot drivers',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'jetcobot_pc_node.py = jetcobot_bringup.nodes.jetcobot_pc_node:main',
        ],
    },
)
