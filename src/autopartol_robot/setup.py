from glob import glob

from setuptools import find_packages, setup

package_name = "autopartol_robot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", ["config/patrol_config.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xinxiyuan",
    maintainer_email="Creek_xin@163.com",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "partol_node=autopartol_robot.partol_node:main",  # 节点名=功能包名.文件名：函数名
            "speaker=autopartol_robot.speaker:main",
        ],
    },
)
