from glob import glob

from setuptools import setup

package_name = "allsense_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    # "allsense" (otterbridge-allsense-python) is installed with pip, not declared here: ROS console scripts
    # check declared requirements at startup and would fail with an unclear error.
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wen-da-ng",
    description="Publishes sensors from an AllSense phone as standard ROS 2 messages.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["bridge = allsense_ros2.bridge:main"]},
)
