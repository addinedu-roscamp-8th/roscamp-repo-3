from setuptools import find_packages, setup

package_name = 'jetcobot_hw_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='jetcobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'joint_states_publisher = jetcobot_hw_bringup.joint_states_publisher:main',
        'trajectory_executor = jetcobot_hw_bringup.trajectory_executor:main',
        'jetcobot_hw_node = jetcobot_hw_bringup.jetcobot_hw_node:main',
        'jetcobot_hw_node_traj = jetcobot_hw_bringup.jetcobot_hw_node_traj:main',
        'jetcobot_hw = jetcobot_hw_bringup.jetcobot_hw:main',
    ],
},


)