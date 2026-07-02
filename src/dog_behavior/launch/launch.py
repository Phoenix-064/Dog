import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            point_lio_launch_file = "mapping_mid360.launch.py" if livox_model == "mid360" else "mapping_horizon.launch.py"
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([point_lio_share, "/launch/", point_lio_launch_file]),
                    launch_arguments={"rviz": use_point_lio_rviz}.items(),
                )
            )
        except Exception:
            actions.append(LogInfo(msg="[dog_behavior.launch] point_lio not found, skip point_lio launch."))

    return actions


def _create_behavior_node(context, *args, **kwargs):
    match_type = LaunchConfiguration("match_type").perform(context)
    goal_frame_id = LaunchConfiguration("goal_frame_id").perform(context)
    test_mode = LaunchConfiguration("test_mode").perform(context).lower() == "true"
    pkg_share = get_package_share_directory("dog_behavior")
    waypoints_file = os.path.join(pkg_share, "config", f"waypoints_{match_type}.yaml")
    tree_xml_file_path = os.path.join(
        pkg_share,
        "config",
        "behavior_tree_nav_serial_test.xml" if test_mode else "behavior_tree.xml",
    )
    log_level = "dog_behavior_bt:=debug" if test_mode else "dog_behavior_bt:=info"

    return [
        Node(
            package="dog_behavior",
            executable="dog_behavior_bt_node",
            name="dog_behavior_bt",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{
                "match_type": match_type,
                "waypoints_file": waypoints_file,
                "goal_frame_id": goal_frame_id,
                "tree_xml_file_path": tree_xml_file_path,
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    # Bring up all first-party and third-party runtime components from one entrypoint.
    test_mode = LaunchConfiguration("test_mode")
    use_point_lio_rviz = LaunchConfiguration("use_point_lio_rviz")
    use_perception_camera = LaunchConfiguration("use_perception_camera")
    use_serial_bridge = LaunchConfiguration("use_serial_bridge")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud_rate = LaunchConfiguration("serial_baud_rate")
    serial_ack_timeout_ms = LaunchConfiguration("serial_ack_timeout_ms")
    use_nav_telemetry_serial = LaunchConfiguration("use_nav_telemetry_serial")
    nav_telemetry_serial_port = LaunchConfiguration("nav_telemetry_serial_port")
    nav_telemetry_baud_rate = LaunchConfiguration("nav_telemetry_baud_rate")
    nav_telemetry_ack_timeout_ms = LaunchConfiguration("nav_telemetry_ack_timeout_ms")
    map_to_camera_x = LaunchConfiguration("map_to_camera_x")
    map_to_camera_y = LaunchConfiguration("map_to_camera_y")
    map_to_camera_z = LaunchConfiguration("map_to_camera_z")
    map_to_camera_roll = LaunchConfiguration("map_to_camera_roll")
    map_to_camera_pitch = LaunchConfiguration("map_to_camera_pitch")
    map_to_camera_yaw = LaunchConfiguration("map_to_camera_yaw")

    declare_test_mode = DeclareLaunchArgument(
        "test_mode",
        default_value="false",
        description="Enable navigation/serial/PointLIO test mode with vision and grasp/place bypassed.",
    )

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

    declare_goal_frame_id = DeclareLaunchArgument(
        "goal_frame_id",
        default_value="map",
        description="Frame id for behavior tree waypoint goals.",
    )

    declare_use_serial_bridge = DeclareLaunchArgument(
        "use_serial_bridge",
        default_value="false",
        description="Whether to start the behavior command serial bridge.",
    )

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for behavior command bridge.",
    )

    declare_serial_baud_rate = DeclareLaunchArgument(
        "serial_baud_rate",
        default_value="115200",
        description="Baud rate for behavior command bridge serial port.",
    )

    declare_serial_ack_timeout_ms = DeclareLaunchArgument(
        "serial_ack_timeout_ms",
        default_value="1500",
        description="Ack timeout in milliseconds for behavior command bridge.",
    )

    declare_use_nav_telemetry_serial = DeclareLaunchArgument(
        "use_nav_telemetry_serial",
        default_value="false",
        description="Whether to start the serial waypoint navigation action node.",
    )

    declare_nav_telemetry_serial_port = DeclareLaunchArgument(
        "nav_telemetry_serial_port",
        default_value="/dev/ttyUSB1",
        description="Serial port for navigation pose telemetry.",
    )

    declare_nav_telemetry_baud_rate = DeclareLaunchArgument(
        "nav_telemetry_baud_rate",
        default_value="115200",
        description="Baud rate for navigation pose telemetry serial port.",
    )

    declare_nav_telemetry_ack_timeout_ms = DeclareLaunchArgument(
        "nav_telemetry_ack_timeout_ms",
        default_value="10000",
        description="Navigation arrival acknowledgement timeout in milliseconds.",
    )

    declare_map_to_camera_x = DeclareLaunchArgument(
        "map_to_camera_x",
        default_value="0.0",
        description="Static TF map->camera_init translation x.",
    )

    declare_map_to_camera_y = DeclareLaunchArgument(
        "map_to_camera_y",
        default_value="0.0",
        description="Static TF map->camera_init translation y.",
    )

    declare_map_to_camera_z = DeclareLaunchArgument(
        "map_to_camera_z",
        default_value="0.0",
        description="Static TF map->camera_init translation z.",
    )

    declare_map_to_camera_roll = DeclareLaunchArgument(
        "map_to_camera_roll",
        default_value="0.0",
        description="Static TF map->camera_init roll (rad).",
    )

    declare_map_to_camera_pitch = DeclareLaunchArgument(
        "map_to_camera_pitch",
        default_value="0.0",
        description="Static TF map->camera_init pitch (rad).",
    )

    declare_map_to_camera_yaw = DeclareLaunchArgument(
        "map_to_camera_yaw",
        default_value="0.0",
        description="Static TF map->camera_init yaw (rad).",
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
        condition=UnlessCondition(test_mode),
    )

    perception_camera_node = Node(
        package="dog_perception",
        executable="dog_perception_camera_node",
        name="dog_perception_camera",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'",
            use_perception_camera,
            "' == 'true' and '",
            test_mode,
            "' != 'true'",
        ])),
    )

    lifecycle_node = Node(
        package="dog_lifecycle",
        executable="dog_lifecycle_node",
        name="dog_lifecycle",
        output="screen",
        parameters=[{
            "valid_frame_topic": "/target/target_3d",
            "heartbeat_timeout_ms": ParameterValue(
                PythonExpression(["600000 if '", test_mode, "' == 'true' else 2000"]),
                value_type=int,
            ),
        }],
    )

    serial_bridge_node = Node(
        package="dog_serial_bridge",
        executable="dog_serial_bridge_node",
        name="dog_serial_bridge",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'",
            use_serial_bridge,
            "' == 'true' and '",
            test_mode,
            "' != 'true'",
        ])),
        parameters=[{
            "serial_port": serial_port,
            "baud_rate": ParameterValue(serial_baud_rate, value_type=int),
            "ack_timeout_ms": ParameterValue(serial_ack_timeout_ms, value_type=int),
        }],
    )

    nav_telemetry_serial_node = Node(
        package="dog_serial_bridge",
        executable="dog_serial_bridge_nav_telemetry_node",
        name="dog_serial_bridge_nav_telemetry",
        output="screen",
        condition=IfCondition(use_nav_telemetry_serial),
        parameters=[{
            "serial_port": nav_telemetry_serial_port,
            "baud_rate": ParameterValue(nav_telemetry_baud_rate, value_type=int),
            "ack_timeout_ms": ParameterValue(nav_telemetry_ack_timeout_ms, value_type=int),
            "send_shutdown_arrival_on_exit": ParameterValue(
                PythonExpression(["'", test_mode, "' == 'true'"]),
                value_type=bool,
            ),
        }],
    )

    static_map_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_camera_init_static_tf",
        output="screen",
        arguments=[
            map_to_camera_x,
            map_to_camera_y,
            map_to_camera_z,
            map_to_camera_roll,
            map_to_camera_pitch,
            map_to_camera_yaw,
            "map",
            "camera_init",
        ],
    )

    behavior_node = OpaqueFunction(function=_create_behavior_node)

    return LaunchDescription([
        declare_test_mode,
        declare_use_livox,
        declare_livox_model,
        declare_use_point_lio,
        declare_use_point_lio_rviz,
        declare_use_perception_camera,
        declare_goal_frame_id,
        declare_use_serial_bridge,
        declare_serial_port,
        declare_serial_baud_rate,
        declare_serial_ack_timeout_ms,
        declare_use_nav_telemetry_serial,
        declare_nav_telemetry_serial_port,
        declare_nav_telemetry_baud_rate,
        declare_nav_telemetry_ack_timeout_ms,
        declare_map_to_camera_x,
        declare_map_to_camera_y,
        declare_map_to_camera_z,
        declare_map_to_camera_roll,
        declare_map_to_camera_pitch,
        declare_map_to_camera_yaw,
        declare_match_type,
        third_party_actions,
        perception_node,
        perception_camera_node,
        lifecycle_node,
        serial_bridge_node,
        nav_telemetry_serial_node,
        static_map_camera_tf,
        behavior_node,
    ])
