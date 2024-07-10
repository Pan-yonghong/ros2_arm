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

"""Launch Webots Assembly Robot simulation nodes."""

import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.actions import LogInfo

PACKAGE_NAME = 'assembly_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)  
    world = LaunchConfiguration('world') 
    robot_description_path = os.path.join(package_dir, 'resource', 'asmb_robot.urdf')   
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_controllers.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # 初始化Webots启动器：创建WebotsLauncher实例，用于启动Webots仿真。
    # 并且设置了ros2_supervisor为True，意味着Webots中的ROS2 Supervisor会被启用
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # Define your URDF robots here
    # The name of an URDF robot has to match the name of the robot of the driver node
    # You can specify the URDF file to use with "urdf_path"
    # spawn_URDF_asmbrobot = URDFSpawner(
    #     name='asmb_robot',              #此处修改模型名
    #     urdf_path=robot_description_path,
    #     translation='0 0 0.01',         #结合urdf文件中虚拟节点调整模型位置
    #     rotation='0 0 1 -1.5708',
    # )

    # Driver nodes
    # When having multiple robot it is mandatory to specify the robot name.
    asmb_robot_driver = WebotsController(
        robot_name='asmb_robot',
        namespace='asmbrobot',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            ros2_control_params
        ],
        respawn=True
    )

    # asmb_robot_process = Node(
    #     package='assembly_robot',
    #     executable='asmb_robot_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #     ],
    # )

    # Other ROS 2 nodes
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    # 启动右臂轨迹控制器，控制机械臂的运动轨迹
    right_arm_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['right_arm_controller', '-c', 'asmbrobot/controller_manager'] + controller_manager_timeout,
    )

    left_arm_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['left_arm_controller', '-c', 'asmbrobot/controller_manager'] + controller_manager_timeout,
    )

    # 启动关节状态广播器，用于发布机器人的关节状态信息
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['asm_joint_state_broadcaster', '-c', 'asmbrobot/controller_manager'] + controller_manager_timeout,
    )
    ros_control_spawners = [right_arm_trajectory_controller_spawner,
                            left_arm_trajectory_controller_spawner,
                            joint_state_broadcaster_spawner]

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

    # Wait for the simulation to be ready to start the tools and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=asmb_robot_driver,
        nodes_to_start= ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='asmb_robot.wbt',
            description='Choose one of the world files from `/assembly_robot/worlds` directory'
        ),
        webots,                 
        webots._supervisor,
        # Request to spawn the URDF robot
        # spawn_URDF_asmbrobot, 

        # Other ROS 2 nodes
        robot_state_publisher,
        # right_arm_trajectory_controller_spawner,
        # left_arm_trajectory_controller_spawner,
        # joint_state_broadcaster_spawner,

        asmb_robot_driver,
        # asmb_robot_process,
        waiting_nodes,

        # Launch the driver node once the URDF robot is spawned
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                on_stdout=lambda event: get_webots_driver_node(event, asmb_robot_driver),
            )
        ),

        # Kill all the nodes when the driver node is shut down
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=asmb_robot_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
