from setuptools import find_packages, setup

package_name = "robocollector_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/controller_node.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maher",
    maintainer_email="mahersabbagh80@gmail.com",
    description="RoboCollector state machine controller for coordinating block collection",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "controller_node = robocollector_controller.controller_node:main"
        ],
    },
)
