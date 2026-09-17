import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #获取功能包share 目录
    urdf_total_path = get_package_share_directory('fishbot_description')


    #获取机器人模型的路径
    default_urdf_path=os.path.join(urdf_total_path,'urdf',"fishbot/fishbot.urdf.xacro")
    
    #default_rviz_path = os.path.join(urdf_total_path,"config","rviz/display_robot_model.rviz")
    #获取世界（地图）路径
    default_gazebo_world_path = os.path.join(urdf_total_path,"world","custom_room.world")

    #声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='URDF的绝对路径'
    )

    #通过文件路径，获取内容，并转换成参数对象，以供传入robot_state_publisher
    substitutions_command_result = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')]),value_type=str
    )

    '''robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result,value_type=str
    )'''
    #状态发布节点
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description':substitutions_command_result}
        ]
    )

    #rviz节点
    '''action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',default_rviz_path]
    )'''
    
    action_launch_gazebo=launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch',"/gazebo.launch.py"]
        ),
        launch_arguments=[('world',default_gazebo_world_path),('verbose','true')]
    )

    #urdf转sdf
    action_spawn_entity =  launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'fishbot',
        ],
        output='screen'
    )


    action_load_joint_state_controller=launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )


    action_load_fishbot_effort_controller=launch.actions.ExecuteProcess(
            cmd='ros2 control load_controller fishbot_effort_controller --set-state active'.split(' '),
            output='screen'
        )
    action_load_diff_drive_controller=launch.actions.ExecuteProcess(
                cmd='ros2 control load_controller fishbot_diff_drive_controller --set-state active'.split(' '),
                output='screen'
            )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
                    event_handler=launch.event_handlers.OnProcessExit(
                        target_action=action_spawn_entity,
                        on_exit=[action_load_diff_drive_controller],
                    )
                )
    ])
