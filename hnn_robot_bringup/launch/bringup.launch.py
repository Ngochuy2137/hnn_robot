from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # ==== 0) Launch args ====
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_x_init_cmd = DeclareLaunchArgument('x_init', default_value='-2.0', description='Robot initial X')
    declare_y_init_cmd = DeclareLaunchArgument('y_init', default_value='-0.5', description='Robot initial Y')

    pkg_share = get_package_share_directory('hnn_robot_description')
    pkg_control_share = get_package_share_directory('hnn_robot_control')
    declare_ros2_ctrl_yaml = DeclareLaunchArgument(
        'ros2_ctrl_yaml',
        default_value=PathJoinSubstitution([pkg_control_share, 'config', 'ros2_controllers.yaml']),
        description='Path to ros2_control controllers yaml'
    )
    declare_use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to use Gazebo simulation'
    )
    declare_robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='hnn_diff_robot',
        choices=['hnn_diff_robot'],
        description='Robot type identifier (hnn_diff_robot, ...)'
    )

    # ==== 1) Paths & LaunchConfigurations ====
    x_init = LaunchConfiguration('x_init')
    y_init = LaunchConfiguration('y_init')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ros2_ctrl_yaml = LaunchConfiguration('ros2_ctrl_yaml')
    use_gazebo = LaunchConfiguration('use_gazebo')
    robot_type = LaunchConfiguration('robot_type')

    pkg_bringup_share = get_package_share_directory('hnn_robot_bringup')
    world_model = PathJoinSubstitution([pkg_bringup_share, 'worlds', 'turtlebot3_world.world'])
    robot_xacro_model = PathJoinSubstitution([pkg_share, 'xacro', 'hnn_diff_robot.urdf.xacro'])
    bridge_params = PathJoinSubstitution([pkg_bringup_share, 'config', 'hnn_diff_robot_bridge.yaml'])
    ros2_controllers_cfg = PathJoinSubstitution([pkg_control_share, 'config', 'ros2_controllers.yaml'])

    # ==== 2) robot_description from Xacro ====
    robot_description_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        robot_xacro_model, ' ',
        'ros2_ctrl_yaml:=', ros2_ctrl_yaml, ' ',
        'use_gazebo:=', use_gazebo, ' ',
        'robot_type:=', robot_type
    ])

    # ==== 3) robot_state_publisher ====
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_cmd, value_type=str)
        }]
    )

    # ==== 4) Gazebo Harmonic (gz sim) ====
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -s -v2 '), world_model],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': TextSubstitution(text='-g -v2'),
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ==== 5) Spawn robot ====
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hnn_bot',
        output='screen',
        arguments=[
            '-world', 'default',
            '-name', 'hnn_bot',
            '-param', 'robot_description',
            '-x', x_init,
            '-y', y_init,
        ],
        parameters=[{
            'robot_description': ParameterValue(robot_description_cmd, value_type=str)
        }]
    )

    # ==== 6) ros_gz_bridge ====
    ros_gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen',
    )

    # ==== 7) Controllers spawners ====
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', ros2_controllers_cfg,
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    spawner_diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', ros2_controllers_cfg,
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    jsb_after_spawn_cmd = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_robot_cmd,
            on_start=[TimerAction(period=1.0, actions=[spawner_jsb])]
        )
    )

    diff_after_jsb_cmd = RegisterEventHandler(
        OnProcessStart(
            target_action=spawner_jsb,
            on_start=[TimerAction(period=1.0, actions=[spawner_diff])]
        )
    )

    return LaunchDescription([
        # Args
        use_sim_time_arg,
        declare_x_init_cmd,
        declare_y_init_cmd,
        declare_ros2_ctrl_yaml,
        declare_use_gazebo_arg,
        declare_robot_type_arg,

        SetParameter(name='use_sim_time', value=use_sim_time),

        # Gazebo
        gzserver_cmd,
        gzclient_cmd,

        # Robot & bridges
        robot_state_publisher_cmd,
        spawn_robot_cmd,
        ros_gz_bridge_cmd,

        # Controllers
        jsb_after_spawn_cmd,
        diff_after_jsb_cmd,
    ])