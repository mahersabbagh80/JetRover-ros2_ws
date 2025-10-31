from setuptools import setup

package_name = "robocollector_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/bringup_robocollector_navigation.launch.py",
                "launch/bringup_robocollector_localization.launch.py",
                "launch/robocollector.launch.py",
                "launch/simulation.launch.py",
                "launch/gazebo_simulation.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/worlds",
            ["worlds/robocollector.world"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maher",
    maintainer_email="maher@example.com",
    description="Bringup and launch orchestration for JetRover RoboCollector.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={},
)
