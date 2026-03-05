from setuptools import setup
import os
from glob import glob

package_name = "drone_docking_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/drone_docking_sim"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/scripts", glob("scripts/*")),
    ],
    entry_points={
        "console_scripts": [
            "two_drones_script = drone_docking_sim.two_drones_script:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sebgra518",
    maintainer_email="sebgra518@gmail.com",
    description="Two-drone ArduPilot + Gazebo Harmonic sim bringup",
    license="MIT",
)