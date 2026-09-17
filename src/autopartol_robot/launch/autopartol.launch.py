import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory  # 获取功功能包的安装目录


def generate_launch_description():
    # 获取默认路径（参数文件安装在 autopartol_robot 功能包的 share 目录下）
    autopartol_robot_path = get_package_share_directory("autopartol_robot")
    default_partol_config_path = os.path.join(
        autopartol_robot_path, "config", "patrol_config.yaml"
    )
    action_partol_node = launch_ros.actions.Node(
        package="autopartol_robot",
        executable="partol_node",
        output="screen",
        parameters=[default_partol_config_path],
    )
    action_speaker_node = launch_ros.actions.Node(
        package="autopartol_robot",
        executable="speaker",
        output="screen",
    )
    return launch.LaunchDescription([action_partol_node, action_speaker_node])
