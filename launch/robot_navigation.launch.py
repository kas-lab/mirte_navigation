import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='use_sim_time'
    )

    nav_to_pose_tree_arg = DeclareLaunchArgument(
        "nav_to_pose_tree",
        default_value="navigate_to_pose_w_replanning_and_recovery.xml",
        description="Which Nav2 navigate_to_pose BT to use",
    )

    nav_through_poses_tree_arg = DeclareLaunchArgument(
        "nav_through_poses_tree",
        default_value="navigate_through_poses_w_replanning_and_recovery.xml",
        description="Which Nav2 navigate_through_poses BT to use",
    )

    # Get the package share directory
    pkg_mirte_navigation = get_package_share_directory('mirte_navigation')

    # Define relative paths for the map and params file
    map_file = os.path.join(
        pkg_mirte_navigation,
        'maps',
        'map.yaml')

    params_file = os.path.join(
        pkg_mirte_navigation,
        'params',
        'mirte_nav2_params.yaml')

    trees_path = os.path.join(
        get_package_share_directory("mirte_navigation"),
        "trees"
    )

    nav2_params_file = ReplaceString(
        source_file=params_file,
        replacements={
            "<nav_to_pose_tree_path>": (
                trees_path,
                os.path.sep,
                LaunchConfiguration("nav_to_pose_tree"),
            ),
            "<nav_through_poses_tree_path>": (
                trees_path,
                os.path.sep,
                LaunchConfiguration("nav_through_poses_tree"),
            ),
        },
    )

    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"
        ])),
        launch_arguments={'map': map_file, 'use_sim_time': use_sim_time}.items()
    )

    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
        ])),
        launch_arguments={
            "map": map_file,
            "params_file": nav2_params_file,
            'use_sim_time': 'true',
        }.items()
    )

    rviz_file = PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"
        ])
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file, "--ros-args", "--log-level", "WARN"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    initial_pose_node = TimerAction(
        period=1.0,  # Delay in seconds before starting the initial pose node
        actions=[
            Node(
                package='mirte_navigation',
                executable='set_initial_pose',
                name='set_initial_pose',
                output='screen',
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )
    return LaunchDescription([
        use_sim_time_arg,
        nav_to_pose_tree_arg,
        nav_through_poses_tree_arg,
        localization_launch,
        initial_pose_node,
        navigation_launch,
        start_rviz_cmd
    ])
