#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path

def generate_launch_description():
    ld = LaunchDescription()

    enable_drive = LaunchConfiguration("enable_drive", default="true")
    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive", default_value="true", description="Enable robot drive node"
    )

    gz_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[
                                                    EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                                                                        default_value=''),
                                                    '/usr/share/gazebo-11/models/:',
                                                    str(Path(get_package_share_directory('jackal_description')).
                                                        parent.resolve())])
    
    jackal_multi_robot = get_package_share_directory("jackal_multi_robot")

    # Legacy Turtlebot3 world file (package renamed to jackal_mutli_robot)
    world = os.path.join(
        jackal_multi_robot, "worlds", "multi_empty_world.sdf"
    )

    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'control.yaml']
    )
    

    # Gazebo server launch file
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py")
        ),
        launch_arguments={"ign_args": '-r ' + world}.items(),
    )


    
    ld.add_action(gz_resource_path)
    ld.add_action(declare_enable_drive)
    ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Spawn jackal instance in gazebo

    # Get URDF via xacro
    robot_description_command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
        ),
        ' ',
        'is_sim:=true',
        ' ',
        'gazebo_controllers:=',
        config_jackal_velocity_controller,
    ]

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )

    # Create state publisher node for that instance
    jackal_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False, # This was set to false in the original implementation, why?
                        "publish_frequency": 10.0,
                        'robot_description': robot_description_content}],
        remappings=remappings,
    )

    # Create spawn call
    spawn_jackal = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-topic",
            f"robot_description",
            "-name",
            "single_jackal",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.51",
            "-Y",
            "3.14159",
            "-unpause",
        ],
        output="screen",
    )


    ld.add_action(jackal_state_publisher)
    ld.add_action(spawn_jackal)

    last_action = spawn_jackal



    # Get URDF via xacro
    robot_description_command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
        ),
        ' ',
        'is_sim:=true',
        ' ',
        'gazebo_controllers:=',
        config_jackal_velocity_controller,
    ]
    
    config_jackal_velocity_controller = PathJoinSubstitution(
            [FindPackageShare('jackal_control'),
            'config',
            'control.yaml'],
        )
    
    # # Add the velocity_controller spawner
    # velocity_controller_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     name='velocity_controller_spawner',
    #     output='screen',
    #     arguments=['jackal_velocity_controller'],
    #     parameters=[config_jackal_velocity_controller]
    # )


    # # Add the joint_state_broadcaster spawner
    # joint_state_broadcaster_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     name='joint_state_broadcaster_spawner',
    #     output='screen',
    #     arguments=['joint_state_broadcaster'],
    #     parameters=[config_jackal_velocity_controller]
    # )

    
    # Launch jackal_control/control.launch.py
    launch_jackal_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('jackal_control'), 'launch', 'control.launch.py']
            )),
            launch_arguments=[('robot_description_command', robot_description_command),
                            ('is_sim', 'True')]
        )
    
    
    # Launch jackal_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_jackal_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py']))
        )

    control_jackal_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_action,
            on_exit=[launch_jackal_control,
                # velocity_controller_node, joint_state_broadcaster_node],
                     launch_jackal_teleop_base],
        )
    )
    ld.add_action(control_jackal_event)

    return ld
