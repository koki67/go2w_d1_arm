from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("go2w_d1_arm"), "config", "d1_arm_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("network_interface", default_value=""),
            DeclareLaunchArgument("enable_on_start", default_value="false"),
            DeclareLaunchArgument("zero_on_start", default_value="false"),
            DeclareLaunchArgument("require_enable_before_motion", default_value="true"),
            DeclareLaunchArgument("multi_joint_mode", default_value="1"),
            DeclareLaunchArgument("lock_force", default_value="80000"),
            Node(
                package="go2w_d1_arm",
                executable="d1_arm_bridge_node",
                name="d1_arm_bridge",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "network_interface": LaunchConfiguration("network_interface"),
                        "enable_on_start": LaunchConfiguration("enable_on_start"),
                        "zero_on_start": LaunchConfiguration("zero_on_start"),
                        "require_enable_before_motion": LaunchConfiguration(
                            "require_enable_before_motion"
                        ),
                        "multi_joint_mode": LaunchConfiguration("multi_joint_mode"),
                        "lock_force": LaunchConfiguration("lock_force"),
                    },
                ],
            ),
        ]
    )

