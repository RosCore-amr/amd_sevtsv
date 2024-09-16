from setuptools import find_packages, setup
import os
from glob import glob

package_name = "amd_sevtsv"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mm",
    maintainer_email="engineer.pqm@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "query_db = amd_sevtsv.query_db:main",
            "mockup = amd_sevtsv.mockup_rosbotStatus:main",
            "robot_control = amd_sevtsv.robot_control:main",
            "mission_control = amd_sevtsv.mission_control:main",
        ],
    },
)
