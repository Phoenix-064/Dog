import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_optional_third_party_actions(context, *args, **kwargs):
    del args
    del kwargs

    actions = []
    use_livox = LaunchConfiguration("use_livox").perform(context).lower() == "true"
    livox_model = LaunchConfiguration("livox_model").perform(context)
    use_point_lio = LaunchConfiguration("use_point_lio").perform(context).lower() == "true"
    use_point_lio_rviz = LaunchConfiguration("use_point_lio_rviz").perform(context)

    if use_livox:
        try:
            livox_share = get_package_share_directory("livox_ros_driver2")
            launch_file = "msg_MID360_launch.py" if livox_model == "mid360" else "msg_HAP_launch.py"
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([livox_share, "/launch_ROS2/", launch_file])
                )
            )
        except Exception:
            actions.append(LogInfo(msg="[dog_behavior.launch] livox_ros_driver2 not found, skip Livox launch."))

    if use_point_lio:
        try:
            point_lio_share = get_package_share_directory("point_lio")
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([point_lio_share, "/launch/point_lio.launch.py"]),
                    launch_arguments={"rviz": use_point_lio_rviz}.items(),
                )
            )
        except Exception:
            actions.append(LogInfo(msg="[dog_behavior.launch] point_lio not found, skip point_lio launch."))

    return actions


def _create_behavior_node(context, *args, **kwargs):
    match_type = LaunchConfiguration("match_type").perform(context)
    pkg_share = get_package_share_directory("dog_behavior")
    waypoints_file = os.path.join(pkg_share, "config", f"waypoints_{match_type}.yaml")

    return [
        Node(
            package="dog_behavior",
            executable="dog_behavior_node",
            name="dog_behavior",
            output="screen",
            parameters=[{
                "match_type": match_type,
                "waypoints_file": waypoints_file,
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    # Bring up all first-party and third-party runtime components from one entrypoint.
    use_point_lio_rviz = LaunchConfiguration("use_point_lio_rviz")
    use_perception_camera = LaunchConfiguration("use_perception_camera")

    declare_use_livox = DeclareLaunchArgument(
        "use_livox",
        default_value="true",
        description="Whether to start Livox ROS2 driver.",
    )

    declare_livox_model = DeclareLaunchArgument(
        "livox_model",
        default_value="mid360",
        choices=["mid360", "hap"],
        description="Livox sensor model launch profile.",
    )

    declare_use_point_lio = DeclareLaunchArgument(
        "use_point_lio",
        default_value="true",
        description="Whether to start point_lio mapping node.",
    )

    declare_use_point_lio_rviz = DeclareLaunchArgument(
        "use_point_lio_rviz",
        default_value="false",
        description="Whether point_lio should start RViz.",
    )

    declare_use_perception_camera = DeclareLaunchArgument(
        "use_perception_camera",
        default_value="false",
        description="Whether to start dog_perception_camera_node.",
    )

    declare_match_type = DeclareLaunchArgument(
        "match_type",
        default_value="left",
        choices=["left", "right"],
        description="比赛类型，决定加载哪组导航坐标",
    )

    third_party_actions = OpaqueFunction(function=_create_optional_third_party_actions)

    perception_node = Node(
        package="dog_perception",
        executable="dog_perception_node",
        name="dog_perception",
        output="screen",
    )

    perception_camera_node = Node(
        package="dog_perception",
        executable="dog_perception_camera_node",
        name="dog_perception_camera",
        output="screen",
        condition=IfCondition(use_perception_camera),
    )

    lifecycle_node = Node(
        package="dog_lifecycle",
        executable="dog_lifecycle_node",
        name="dog_lifecycle",
        output="screen",
    )

    behavior_node = OpaqueFunction(function=_create_behavior_node)

    navigation_executor_node = Node(
        package="dog_behavior",
        executable="dog_behavior_navigation_executor_node",
        name="dog_navigation_executor",
        output="screen",
    )

    return LaunchDescription([
        declare_use_livox,
        declare_livox_model,
        declare_use_point_lio,
        declare_use_point_lio_rviz,
        declare_use_perception_camera,
        declare_match_type,
        third_party_actions,
        perception_node,
        perception_camera_node,
        lifecycle_node,
        behavior_node,
        navigation_executor_node,
    ])
