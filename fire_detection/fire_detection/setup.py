from glob import glob
import os

from setuptools import setup

package_name = "top_view_fire"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="addinedu",
    maintainer_email="addinedu@todo.com",
    description="Top-view fire detector that publishes map coordinates as ROS topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fire_map_publisher = top_view_fire.fire_map_publisher_node:main",
        ],
    },
)
