from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Motion Planning API Tutorial node
    motion_planning_api_tutorial = Node(
        package="moveit_tutorial",
        executable="motion_planning_api_tutorial",
        name="motion_planning_api_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"planning_plugin": "ompl_interface/OMPLPlanner"}
        ],
    )

    return LaunchDescription([motion_planning_api_tutorial])