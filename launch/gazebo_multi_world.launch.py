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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    enable_drive = LaunchConfiguration("enable_drive", default="true")
    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive", default_value="true", description="Enable robot drive node"
    )


    jackal_multi_robot = get_package_share_directory("jackal_multi_robot")
    # launch_file_dir = os.path.join(turtlebot3_multi_robot, "launch")

    # Legacy Turtlebot3 world file (package renamed to jackal_mutli_robot)
    world = os.path.join(
        jackal_multi_robot, "worlds", "multi_empty_world.world"
    )

    ## Jackal world file 
    # world = PathJoinSubstitution(
    #     [FindPackageShare('jackal_gazebo'),
    #     'worlds',
    #     'jackal_race.world'],
    # )

    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'control.yaml']
    )
    
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
    
    print("robot_description_command is: ", robot_description_command)

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )
    print("robot_description_content is: ", robot_description_content)
    
    # launch_jackal_description = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution(
    #                 [FindPackageShare('jackal_description'),
    #                  'launch',
    #                  'description.launch.py']
    #             )
    #         ),
    #         launch_arguments=[('robot_description_command', robot_description_command)]
    #     )

    # Gazebo server and client (launch file)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
    )

    # # Gazebo server and client (direct execution)
    # gzserver = ExecuteProcess(
    #     cmd=['gzserver',
    #          '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so',
    #          '--verbose',
    #          world_path],
    #     output='screen',
    # )
    # gzclient = ExecuteProcess(
    #     cmd=['gzclient'],
    #     output='screen',
    # )

    

    ld.add_action(declare_enable_drive)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ROWS = 3
    COLS = 1

    x = -ROWS
    y = -COLS
    last_action = None

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Spawn turtlebot3 instances in gazebo
    for i in range(COLS):
        x = -ROWS
        for j in range(ROWS):
            # Construct a unique name and namespace
            name = "jackal" + str(i) + "_" + str(j)
            namespace = "/jc" + str(i) + "_" + str(j)

            # Create state publisher node for that instance
            jackal_state_publisher = Node(
                package="robot_state_publisher",
                namespace=namespace,
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": False,
                             "publish_frequency": 10.0,
                             'robot_description': robot_description_content}],
                remappings=remappings,
            )

            # Create spawn call
            spawn_jackal = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic",
                    f"{namespace}/robot_description",
                    "-entity",
                    name,
                    "-robot_namespace",
                    namespace,
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    "0.01",
                    "-Y",
                    "3.14159",
                    "-unpause",
                ],
                output="screen",
            )

            # Advance by 2 meter in x direction for next robot instantiation
            x += 2.0

            if last_action is None:
                # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
                ld.add_action(jackal_state_publisher)
                ld.add_action(spawn_jackal)
                
            else:
                # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
                # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
                spawn_jackal_event = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[spawn_jackal,
                                 jackal_state_publisher],
                    )
                )
                ld.add_action(spawn_jackal_event)

            # Save last instance for next RegisterEventHandler
            last_action = spawn_jackal

        # Advance by 2 meter in y direction for next robot instantiation
        y += 2.0

    # Start all driving nodes after the last robot is spawned
    for i in range(COLS):
        for j in range(ROWS):
            namespace = "/tb" + str(i) + "_" + str(j)
            
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
                [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py'])))
            
            #TODO: add the control drives

            
            # # Create spawn call
            # drive_turtlebot3_burger = Node(
            #     package="turtlebot3_gazebo",
            #     executable="turtlebot3_drive",
            #     namespace=namespace,
            #     output="screen",
            #     condition=IfCondition(enable_drive),
            # )

            # # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            # drive_turtlebot3_event = RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=last_action,
            #         on_exit=[drive_turtlebot3_burger],
            #     )
            # )
            
            # ld.add_action(drive_turtlebot3_event)

    return ld
