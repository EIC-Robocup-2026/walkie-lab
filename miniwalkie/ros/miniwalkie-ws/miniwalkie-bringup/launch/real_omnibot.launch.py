#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "robot_bringup"
    package_dir = os.path.join(get_package_share_directory(package_name))
    description_package_name = "walkie_description"

    default_robot = os.path.join(
        get_package_share_directory(description_package_name),
        "robots",
        "gz_walkie.urdf.xacro",
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_model = LaunchConfiguration("robot_model", default=default_robot)
    ros2_control = LaunchConfiguration("ros2_control", default="topic_base")

    robot_description_content = Command(
        ["xacro ", default_robot, " ros2_control:=", ros2_control]
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux",
        "twist_mux.yaml",
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    twist_stamped_frame_id = "base_footprint"
    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        remappings=[
            ("/cmd_vel_in", "/cmd_vel"),
            ("/cmd_vel_out", "/omni_wheel_drive_controller/cmd_vel"),
        ],
        parameters=[
            {"frame_id": twist_stamped_frame_id},
        ],
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_model": robot_model,
            "ros2_control": ros2_control,
        }.items(),
    )

    controllers_config = os.path.join(
        get_package_share_directory(description_package_name),
        "config",
        "ros2_controller",
        "miniwalkie_controller.yml",
    )
    controller_manager_spawner = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description", robot_description_content},
            controllers_config,
        ],
    )

    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--switch-timeout", "30.0"],
    )

    delayed_omni_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[omni_controller_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--switch-timeout", "30.0"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[joint_broad_spawner],
        )
    )

    ld = LaunchDescription()
    # Add the commands to the launch description
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(twist_mux)
    ld.add_action(twist_stamper_node)
    ld.add_action(controller_manager_spawner)
    ld.add_action(delayed_omni_controller_spawner)
    ld.add_action(delayed_joint_broad_spawner)

    return ld
