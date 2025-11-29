import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# HNNBOT_MODEL = os.environ['HNNBOT_MODEL']
HNNBOT_MODEL = 'diff_robot'

def launch_setup(context):
    # Get use_gazebo value from launch arguments
    use_gazebo_str = context.launch_configurations.get('use_gazebo', 'true')
    
    # Automatically set use_sim_time = use_gazebo
    # use_sim_time is only used for simulation, so it should match use_gazebo
    use_sim_time_str = use_gazebo_str
    
    # Update launch_configurations so use_sim_time matches use_gazebo
    context.launch_configurations['use_sim_time'] = use_sim_time_str
    
    # Create LaunchConfigurations for use in substitutions
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default=use_sim_time_str)
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('hnn_robot_bringup'),
            'map',
            'map.yaml'))

    param_file_name = HNNBOT_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('hnn_robot_bringup'),
            'config',
            'navigation_stack',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('hnn_robot_bringup'),
        'rviz',
        'rviz.rviz')

    return [
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use Gazebo simulation. use_sim_time will automatically match this value.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=None,
            description='Use simulation (Gazebo) clock if true. Automatically set from use_gazebo.'),
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])