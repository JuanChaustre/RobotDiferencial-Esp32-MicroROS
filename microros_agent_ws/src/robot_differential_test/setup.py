from setuptools import find_packages, setup

package_name = "robot_differential_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Juan",
    maintainer_email="juan@example.com",
    description="Pruebas ROS 2 para el robot diferencial micro-ROS.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "monitor = robot_differential_test.monitor:main",
            "setpoint = robot_differential_test.setpoint:main",
            "navigate = robot_differential_test.navigation:main",
            "manual = robot_differential_test.manual_control:main",
        ],
    },
)
