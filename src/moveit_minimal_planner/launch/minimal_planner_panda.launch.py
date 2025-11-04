# launch/minimal_planner_with_panda_demo.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from moveit_configs_utils import MoveItConfigsBuilder  # <-- add this import

def generate_launch_description():
    # Build the same Panda MoveIt config to hand to *your* node as parameters
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config")
        .to_moveit_configs()
    )

    # Include the official Panda demo (starts robot_state_publisher, fake controllers, move_group, RViz)
    panda_demo_launch = Path(
        get_package_share_directory("moveit_resources_panda_moveit_config")
    ) / "launch" / "demo.launch.py"
    demo = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(panda_demo_launch)))

    # Your planner node, now with all MoveIt params injected
    planner = Node(
        package="moveit_minimal_planner",
        executable="minimal_planner",
        output="screen",
        parameters=[moveit_config.to_dict(), {"planning_group": "panda_arm"}],
        # optional: name="minimal_planner"
    )

    return LaunchDescription([demo, planner])
