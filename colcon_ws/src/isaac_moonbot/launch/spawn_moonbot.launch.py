import os
import pathlib

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
    isaac_robot_description_path = os.path.join(
        get_package_share_directory('moonbot_description'))

    xacro_file = os.path.join(isaac_robot_description_path,
                              'urdf',
                              'yonaka_0722',
                              'moonbot01.urdf.xacro')
    urdf_path = os.path.join(isaac_robot_description_path, 'urdf', 'yonaka_0722', 'moonbotY.urdf')
    # xacroをロード
    doc = xacro.process_file(xacro_file)
    # # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()
    # # relative_urdf_path = pathlib.Path(urdf_path).relative_to(os.getcwd())
    relative_urdf_path = urdf_path

    # params = {'robot_description': robot_desc}
    params = {'robot_description': doc.toxml(), 'use_sim_time': False}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("isaac_moonbot"),
            "config",
            "isaac_moonbot_jtc.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
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

        
    isaac_spawn_robot = Node(
        package="isaac_ros2_scripts",
        executable="spawn_robot",
        parameters=[{'urdf_path': str(relative_urdf_path),
                    'x' : 0.0,
                    'y' : 0.0,
                    'z' : 1.0,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : 0.0,
                    'fixed' : False,
                    }],
    )

    isaac_prepare_sensors = Node(
        package="isaac_ros2_scripts",
        executable="prepare_sensors",
        parameters=[{'urdf_path': str(relative_urdf_path)}],
    )

    isaac_prepare_robot_controller = Node(
        package="isaac_ros2_scripts",
        executable="prepare_robot_controller",
        parameters=[{'urdf_path': str(relative_urdf_path)}],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=isaac_spawn_robot,
                on_exit=[isaac_prepare_sensors],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=isaac_prepare_sensors,
                on_exit=[isaac_prepare_robot_controller],
            )
        ),
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
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
        isaac_spawn_robot,
    ])