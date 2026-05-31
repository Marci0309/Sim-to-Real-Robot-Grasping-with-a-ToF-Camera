from glob import glob

from setuptools import setup

package_name = "xarm1s_manipulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Marcell Mor Szokedencsi",
    maintainer_email="szokedencsim@gmail.com",
    description="Closed-loop pick-and-place for the xArm 1S sim (Phase 4A).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_executor = xarm1s_manipulation.grasp_executor:main",
        ],
    },
)
