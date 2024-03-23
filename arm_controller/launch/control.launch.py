import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    des_pkg_share = FindPackageShare(package='arm_description').find('arm_description')
    urdf_model_path = os.path.join(des_pkg_share, f'urdf/sagittarius_descriptions.urdf')
    con_pkg_share = FindPackageShare(package='arm_controller').find('arm_controller')

    moveit_config = (
        MoveItConfigsBuilder("sgr", package_name="arm_moveit_config")
        .robot_description(file_path=urdf_model_path)
    ).to_moveit_configs()

    controller = Node(
        name="move_group_interface",
        package="arm_controller",
        executable="controller",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    # # RViz
    # rviz_config_file = (
    #     get_package_share_directory("arm_controller") + "/launch/moveit.rviz"
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.planning_pipelines,
    #         moveit_config.joint_limits,
    #     ],
    # )

    ld = LaunchDescription()
    ld.add_action(controller)

    return ld