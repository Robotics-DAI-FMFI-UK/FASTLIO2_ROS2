import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_cfg = PathJoinSubstitution([FindPackageShare("localizer"), "rviz", "localizer.rviz"])
    localizer_config_path = PathJoinSubstitution([FindPackageShare("localizer"), "config", "localizer.yaml"])
    lio_config_path = PathJoinSubstitution([FindPackageShare("fastlio2"), "config", "lio.yaml"])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start RViz together with localization."),
        launch_ros.actions.Node(
            package="fastlio2", namespace="fastlio2", executable="lio_node",
            name="lio_node", output="screen", parameters=[{"config_path": lio_config_path}]),
        launch_ros.actions.Node(
            package="localizer", namespace="localizer", executable="localizer_node",
            name="localizer_node", output="screen", parameters=[{"config_path": localizer_config_path}]),
        launch_ros.actions.Node(
            package="rviz2", namespace="localizer", executable="rviz2", name="rviz2",
            output="screen", arguments=["-d", rviz_cfg], condition=IfCondition(use_rviz)),
    ])
