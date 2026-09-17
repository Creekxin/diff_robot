import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #获取默认路径
    urdf_total_path = get_package_share_directory('fishbot_description')
    default_urdf_path = urdf_total_path + '/urdf/fishbot/fishbot.urdf.xacro'
    default_rviz_path = urdf_total_path + '/config/rviz/display_robot_model.rviz'
    #为launch 添加参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='URDF的绝对路径'
    )

    #获取文件内容生成新的参数
    '''robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat ',launch.substitutions.LaunchConfiguration('model')]),value_type=str
    )'''
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')]),value_type=str
    )
    #状态发布节点
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description':robot_description}
        ]
    )

    #关节状态发布节点
    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    #rviz节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',default_rviz_path]
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
