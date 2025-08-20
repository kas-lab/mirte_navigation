import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare  
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution 
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import LaunchConfigurationEquals
from nav2_common.launch import ReplaceString

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_map = LaunchConfiguration('use_map')
    real_robot = LaunchConfiguration('real_robot')
    lattice_planner = LaunchConfiguration('lattice_planner')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='use_sim_time'
    )

    use_map_arg = DeclareLaunchArgument(
        'use_map',
        default_value='True',
        description='Run using a premade map, no SLAM (True/False)',
    )

    real_robot_arg = DeclareLaunchArgument(
        'real_robot',
        default_value='True',
        description='Wether the real robot is being used (True/False)',
    )

    lattice_planner_arg = DeclareLaunchArgument(
        'lattice_planner',
        default_value='False',
        description='Wether the lattice planner is being used (True/False)',
    )
    
    # Get the package share directory
    pkg_mirte_navigation = get_package_share_directory('mirte_navigation')

    # Define relative paths for the map and params file
    map_file = os.path.join(
        pkg_mirte_navigation,
        'maps',
        'robocup_sim_mirte.yaml')
    
    simulated_navigation_params_file = os.path.join(
        pkg_mirte_navigation,
        'params',
        'mirte_nav2_params.yaml')
    
    real_navigation_params_file = os.path.join(
        pkg_mirte_navigation,
        'params',
        'real_mirte_nav2_params.yaml')

    navigation_params_file = IfElseSubstitution(
        condition=PythonExpression([
            real_robot
        ]),
        if_value=real_navigation_params_file,
        else_value=simulated_navigation_params_file
    )

    lattice_planner_confg = os.path.join(
        pkg_mirte_navigation,
        'params',
        'lattice_output.json')

    navigation_params_file = IfElseSubstitution(
        condition=PythonExpression([
            lattice_planner
        ]),
        if_value=ReplaceString(
            source_file=navigation_params_file,
            replacements={'<lattice_config_path>': lattice_planner_confg}
        ),
        else_value=navigation_params_file
    )

    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'
        ])),
        launch_arguments={
            'map': map_file,
            'params_file': navigation_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
        ])),
        launch_arguments={
            'map': map_file,
            'params_file': navigation_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    slam_params_file = os.path.join(
        pkg_mirte_navigation,
        'params',
        'mirte_slam_params.yaml'
    )
    slam_tb_path = get_package_share_directory('slam_toolbox')
    slam_tb_launch_path = os.path.join(slam_tb_path, 'launch', 'online_async_launch.py')
    nav2_with_slam = GroupAction(
        actions=[
            navigation_launch,
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(slam_tb_launch_path),
                launch_arguments = {
                    'slam_params_file': slam_params_file,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ],
        condition=LaunchConfigurationEquals('use_map', 'False'),
    )

    nav_and_localization = GroupAction(
        actions=[
            localization_launch,
            navigation_launch
        ],
        condition=LaunchConfigurationEquals('use_map', 'True'),
    )

    rviz_file = PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'
        ])
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file, '--ros-args', '--log-level', 'WARN'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    initial_pose_node = TimerAction(
        period=2.0,  # Delay in seconds before starting the initial pose node
        actions=[
            Node(
                package='mirte_navigation',
                executable='set_initial_pose',
                name='set_initial_pose',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    relay_topic_cmd = Node(
        package = "topic_tools",
        executable = "relay",
        arguments=["/cmd_vel", "/mirte_base_controller/cmd_vel"],
        output="screen",
    )
    relay_topic_odom = Node(
        package = "topic_tools",
        executable = "relay",
        arguments=["/mirte_base_controller/odom", "/odom"],
        output="screen",
    )

    return LaunchDescription([
        use_map_arg,
        use_sim_time_arg,
        real_robot_arg,
        lattice_planner_arg,
        nav_and_localization,
        nav2_with_slam,
        start_rviz_cmd,
        initial_pose_node,
        relay_topic_cmd,
        relay_topic_odom
    ])
