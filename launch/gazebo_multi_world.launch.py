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
    # launch_file_dir = os.path.join(turtlebot3_multi_robot, "launch")

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

    ROWS = 2
    COLS = 1

    x = -ROWS
    y = -COLS
    last_action = None

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Spawn jackal instances in gazebo
    for i in range(COLS):
        x = -ROWS
        for j in range(ROWS):
            # Construct a unique name and namespace
            name = "jackal" + str(i) + "_" + str(j)
            namespace = "jc" + str(i) + "_" + str(j)
            frame_prefix = [namespace, '/']

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
                'prefix:=',  # Pass the namespace as the prefix argument
                namespace,
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
                namespace=namespace,
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": False, # This was set to false in the original implementation, why?
                             "publish_frequency": 10.0,
                             'robot_description': robot_description_content,
                             'frame_prefix': ''.join(frame_prefix)}],
                remappings=remappings,
            )

            # Create spawn call
            spawn_jackal = Node(
                package="ros_ign_gazebo",
                executable="create",
                namespace=namespace,
                arguments=[
                    "-topic",
                    f"robot_description",
                    "-name",
                    name,
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    "0.51",
                    "-Y",
                    "3.14159",
                    "-unpause",
                ],
                output="screen",
            )

            launch_jackal_control = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution(
                        [FindPackageShare('jackal_control'), 'launch', 'control.launch.py']
                    )),
                    launch_arguments=[('robot_description_command', robot_description_command),
                                    ('is_sim', 'True'),
                                    ('namespace', namespace)]
                )
            # Advance by 2 meter in x direction for next robot instantiation
            x += 2.0

            if last_action is None:
                # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
                ld.add_action(jackal_state_publisher)
                ld.add_action(spawn_jackal)
                ld.add_action(launch_jackal_control)

                
            else:
                # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
                # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
                spawn_jackal_event = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[spawn_jackal,
                                 jackal_state_publisher,
                                 launch_jackal_control],
                    )
                )
                ld.add_action(spawn_jackal_event)

            # Save last instance for next RegisterEventHandler
            last_action = spawn_jackal

        # Advance by 2 meter in y direction for next robot instantiation
        y += 2.0

    # # Start controllers after all robots are spawned
    # for i in range(COLS):
    #     for j in range(ROWS):
            
    #         # Construct a unique namespace for each robot
    #         namespace = "jc" + str(i) + "_" + str(j)

    #         # Get URDF via xacro
    #         robot_description_command = [
    #             PathJoinSubstitution([FindExecutable(name='xacro')]),
    #             ' ',
    #             PathJoinSubstitution(
    #                 [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
    #             ),
    #             ' ',
    #             'is_sim:=true',
    #             ' ',
    #             'prefix:=',  # Pass the namespace as the prefix argument
    #             namespace,
    #             ' ',
    #             'gazebo_controllers:=',
    #             config_jackal_velocity_controller,
    #         ]

    #         # # Add the velocity_controller spawner
    #         # velocity_controller_node = Node(
    #         #     package='controller_manager',
    #         #     executable='spawner',
    #         #     name='velocity_controller_spawner',
    #         #     namespace=namespace,
    #         #     output='screen',
    #         #     arguments=['jackal_velocity_controller']
    #         # )

            
    #         # # Add the joint_state_broadcaster spawner
    #         # joint_state_broadcaster_node = Node(
    #         #     package='controller_manager',
    #         #     executable='spawner',
    #         #     name='joint_state_broadcaster_spawner',
    #         #     namespace=namespace,
    #         #     output='screen',
    #         #     arguments=['joint_state_broadcaster']
    #         # )
    #         # Launch jackal_control/control.launch.py
    #         launch_jackal_control = IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(PathJoinSubstitution(
    #                     [FindPackageShare('jackal_control'), 'launch', 'control.launch.py']
    #                 )),
    #                 launch_arguments=[('robot_description_command', robot_description_command),
    #                                 ('is_sim', 'True'),
    #                                 ('namespace', namespace)]
    #             )
    #             # Launch jackal_control/teleop_base.launch.py which is various ways to tele-op
    #         # the robot but does not include the joystick. Also, has a twist mux.
    #         # launch_jackal_teleop_base = IncludeLaunchDescription(
    #         #     PythonLaunchDescriptionSource(PathJoinSubstitution(
    #         #     [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py'])),
    #         #     launch_arguments=[('namespace', namespace)]
    #         #     )

    #         control_jackal_event = RegisterEventHandler(
    #             event_handler=OnProcessExit(
    #                 target_action=last_action,
    #                 on_exit=[launch_jackal_control],
    #                         #  launch_jackal_teleop_base],
    #             )
    #         )
    #         ld.add_action(control_jackal_event)

    return ld
