from setuptools import find_packages, setup
from glob import glob
import os

package_name = "wall_climber"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="pnadan@cmu.edu",
    description="Codebase for the Sally magnetic wall-climbing robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = wall_climber.main:main",
        ],
    },
)
