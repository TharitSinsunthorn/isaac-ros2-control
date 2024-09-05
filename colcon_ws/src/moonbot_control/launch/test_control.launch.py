#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

# this is the function launch  system will look for


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["forward_position_controller", "-c", "/controller_manager"],
    # )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller", "-c", "/controller_manager"],
    )

    # robot_controller = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[/robot_description, <your_controllers_yaml_file_path>]
    # )
    

    RF_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["RFposition_trajectory_controller", "-c", "/controller_manager"],
    )

    LF_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["LFposition_trajectory_controller", "-c", "/controller_manager"],
    )

    LR_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["LRposition_trajectory_controller", "-c", "/controller_manager"],
    )

    RR_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["RRposition_trajectory_controller", "-c", "/controller_manager"],
    )
    
    LA_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["LAposition_trajectory_controller", "-c", "/controller_manager"],
    )

    RA_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["RAposition_trajectory_controller", "-c", "/controller_manager"],
    )


    # create and return launch description object
    return LaunchDescription(
        [
            
            joint_state_broadcaster_spawner,
            # robot_controller_spawner,
            
            RegisterEventHandler(
                event_handler=OnProcessExit(
                  target_action=joint_state_broadcaster_spawner,
                  on_exit=[RF_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=RF_robot_controller_spawner,
                  on_start=[LF_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=LF_robot_controller_spawner,
                  on_start=[LR_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=LR_robot_controller_spawner,
                  on_start=[RR_robot_controller_spawner],
                )
            ),
            
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=RR_robot_controller_spawner,
                  on_start=[LA_robot_controller_spawner],
                )
            ),
            
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=LA_robot_controller_spawner,
                  on_start=[RA_robot_controller_spawner],
                )
            ),
            
        ]
    )