import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare  

def generate_launch_description():
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

    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"
        ])),
        launch_arguments={'map': map_file}.items()
    )

    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
        ])),
        launch_arguments={
            "map": map_file,
            "params_file": params_file
        }.items()
    )

    # RViz execution
    rviz_command = ExecuteProcess(
        cmd=["rviz2", "-d", PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"
        ])],
        output="screen"
    )

    return LaunchDescription([
        localization_launch,
        navigation_launch,
        rviz_command
    ])
