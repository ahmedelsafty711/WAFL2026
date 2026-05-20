from setuptools import setup, find_packages
import os
from glob import glob

package_name = "pallet_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Required for ament resource index
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        # Package manifest
        (os.path.join("share", package_name), ["package.xml"]),
        # Launch files
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        # Optional: ship default parameter YAML files
        # (os.path.join("share", package_name, "config"),
        #  glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your@email.com",
    description="Pallet ArUco detection and autonomous navigation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pallet_pose_node = pallet_vision.pallet_pose_node:main",
            "pallet_navigator = pallet_vision.pallet_navigator:main",
            "tf2_pose_transformer = pallet_vision.tf2_pose_transformer:main",
        ],
    },
)
