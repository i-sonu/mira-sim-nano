import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mira_pybullet_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="adarsh",
    maintainer_email="adarshh3000@gmail.com",
    description="Headless CPU-only PyBullet AUV simulator at the /master interface.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = mira_pybullet_sim.sim_node:main",
            "teleop_keyboard = mira_pybullet_sim.teleop_keyboard:main",
            "scripted_test = mira_pybullet_sim.scripted_test:main",
        ],
    },
)
