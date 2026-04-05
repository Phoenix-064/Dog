from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Bring up the main Dog runtime pipeline in dependency order.
    perception_node = Node(
        package="dog_perception",
        executable="dog_perception_node",
        name="dog_perception",
        output="screen",
    )

    lifecycle_node = Node(
        package="dog_lifecycle",
        executable="dog_lifecycle_node",
        name="dog_lifecycle",
        output="screen",
    )

    behavior_node = Node(
        package="dog_behavior",
        executable="dog_behavior_node",
        name="dog_behavior",
        output="screen",
    )

    return LaunchDescription([
        perception_node,
        lifecycle_node,
        behavior_node,
    ])
