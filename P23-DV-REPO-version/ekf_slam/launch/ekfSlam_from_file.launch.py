import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    slam_dir = get_package_share_directory("ekf_slam")
    ekfSlam_from_file_node = Node(
        package="ekf_slam",
        name="ekfSlam_from_file_node",
        executable="read_files",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(slam_dir, "config", "from_file_params.yaml")]
    )

    path_planning_dir = get_package_share_directory("path_planner")
    path_planning_node = Node(
        package="path_planner",
        name="path_planner",
        executable="path_planner",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(path_planning_dir, "config", "parameters.yaml")]
    )

    ld.add_action(ekfSlam_from_file_node)
    #ld.add_action(path_planning_node)
    return ld