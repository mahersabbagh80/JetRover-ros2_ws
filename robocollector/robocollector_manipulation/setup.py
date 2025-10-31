from setuptools import find_packages, setup

package_name = "robocollector_manipulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pickup_node.launch.py"]),
        ("share/" + package_name + "/config", ["config/pickup_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maher",
    maintainer_email="mahersabbagh80@gmail.com",
    description="RoboCollector manipulation node for arm control and object pickup",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "pickup_node = robocollector_manipulation.pickup_node:main",
            "mock_pickup_node = robocollector_manipulation.mock_pickup_node:main",
        ],
    },
)
