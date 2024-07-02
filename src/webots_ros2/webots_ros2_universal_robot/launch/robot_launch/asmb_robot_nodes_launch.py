#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots Universal Robot simulation nodes."""

import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_controller import WebotsController
from launch.actions import LogInfo

PACKAGE_NAME = 'assembly_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    robot_description_path = os.path.join(package_dir, 'urdf', 'asmb_robot.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config_asmb_robot.yaml')

    # Define your URDF robots here
    # The name of an URDF robot has to match the name of the robot of the driver node
    # You can specify the URDF file to use with "urdf_path"
    spawn_URDF_asmbrobot = URDFSpawner(
        name='asmb_robot', #此处修改模型名
        urdf_path=robot_description_path,
        translation='0 0 0.6',
        rotation='0 0 1 -1.5708',
    )

    # Driver nodes
    # When having multiple robot it is mandatory to specify the robot name.
    universal_robot_driver = WebotsController(
        robot_name='asmb_robot',
        namespace='asmbrobot',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            ros2_control_params
        ],
    )

    # Other ROS 2 nodes
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    # 负责启动轨迹控制器，控制机械臂的运动轨迹
    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller', '-c', 'asmbrobot/controller_manager'] + controller_manager_timeout,
    )

    # 启动关节状态广播器，用于发布机器人的关节状态信息
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster', '-c', 'asmbrobot/controller_manager'] + controller_manager_timeout,
    )

    # 发布机器人的整体状态，包括TF变换
    robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace='asmbrobot',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    return LaunchDescription([
        # Request to spawn the URDF robot
        spawn_URDF_asmbrobot, 
        # Other ROS 2 nodes
        robot_state_publisher,
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,

        # Launch the driver node once the URDF robot is spawned
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_URDF_asmbrobot,
                on_stdout=lambda event: get_webots_driver_node(event, universal_robot_driver),
            )
        ),

        # Kill all the nodes when the driver node is shut down
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=universal_robot_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
