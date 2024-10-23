import os
import warnings

from glob import glob
from setuptools import find_packages, setup

# Suppress EasyInstallDeprecationWarning
warnings.filterwarnings("ignore")

package_name = "project_main"

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
        (os.path.join("lib", package_name), glob(package_name + "/*_utils.py*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fede3751, EdoardoAllegrini",
    maintainer_email="trombetti@di.uniroma1.it, allegrini.1969146@studenti.uniroma1.it",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simulation_manager=project_main.simulation_manager:main",
            "balloon_controller=project_main.balloon_controller:main",
            "sensor_controller=project_main.sensor_controller:main",
            "movement_coordinator=project_main.movement_coordinator:main",
            "base_station_controller=project_main.base_station_controller:main",
        ],
    },
)
