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

"""Launch Webots Universal Robot simulation world."""

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_launcher import WebotsLauncher   # webots_ros2_driver包中的WebotsLauncher类，用于启动Webots

PACKAGE_NAME = 'assembly_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)     # 获取该ROS2包的共享目录路径
    world = LaunchConfiguration('world')                        # 创建一个LaunchConfiguration对象，名为world

    # 初始化Webots启动器：创建WebotsLauncher实例，用于启动Webots仿真。
    webots = WebotsLauncher(
        # 它指定了要加载的世界文件路径（通过拼接package_dir、worlds/和world配置得到）
        # 并且设置了ros2_supervisor为True，意味着Webots中的ROS2 Supervisor会被启用
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='asmb_robot.wbt',
            description='Choose one of the world files from `/assembly_robot/worlds` directory'
        ),
        webots,                 # 添加之前定义的 webots 实例到启动序列中
        webots._supervisor,
        # This action will kill all nodes once the Webots simulation has exited
        # 当Webots进程退出时，注册的事件处理器会触发，发送一个Shutdown事件，导致所有由当前launch文件启动的节点关闭，确保干净的退出过程。
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
