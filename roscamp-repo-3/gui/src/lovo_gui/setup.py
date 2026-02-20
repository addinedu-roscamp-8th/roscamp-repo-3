from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lovo_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['lovo_gui/config/domain_bridge_config.yaml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={
        'lovo_gui': ['config/*.yaml', 'config/*.json'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='j01028957795@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lovo_gui = lovo_gui.main_window:main'
        ],
    },
)
