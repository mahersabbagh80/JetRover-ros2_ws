from setuptools import find_packages, setup

package_name = "robocollector_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/vision_node.launch.py"]),
        ("share/" + package_name + "/config", ["config/vision_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maher",
    maintainer_email="mahersabbagh80@gmail.com",
    description="RoboCollector vision node for color-based block detection",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["vision_node = robocollector_vision.vision_node:main"],
    },
)
