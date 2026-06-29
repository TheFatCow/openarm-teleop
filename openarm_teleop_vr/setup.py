import os
from glob import glob

from setuptools import find_packages, setup

package_name = "openarm_teleop_vr"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "isaac"), glob("isaac/*.py")),
        (os.path.join("share", package_name, "vr"), glob("vr/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenArm Teleop",
    maintainer_email="kevin.li@tnmathcoalition.org",
    description="VR teleoperation for the OpenArm bimanual robot (open Pinocchio IK)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "openarm_teleop_vr_node = openarm_teleop_vr.openarm_teleop_vr_node:main",
            "openarm_isaac_relay = openarm_teleop_vr.isaac_relay_node:main",
        ],
    },
)
