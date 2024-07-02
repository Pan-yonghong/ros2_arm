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
#                      为ABB IRB4600机器人创建一个轨迹跟踪客户端，用于多机器人演示
"""Trajectory follower client for the ABB irb4600 robot used for multi-robot demonstration."""

import rclpy

# 从指定模块导入 FollowJointTrajectoryClient 类，用于发送关节轨迹目标给机器人控制器。
from webots_ros2_universal_robot.follow_joint_trajectory_client import FollowJointTrajectoryClient

#定义了一个字典GOAL，里面包含了机器人的关节轨迹信息。具体来说：
#'joint_names' 列出了参与轨迹控制的关节名称。
#'points' 是一系列轨迹点，每个点定义了各个关节在特定时间点的位置和到达该点所需的时间。
GOAL = {
    'joint_names': [
        'A motor',
        'B motor',
        'C motor',
        'E motor',
        'finger_1_joint_1',
        'finger_2_joint_1',
        'finger_middle_joint_1'
    ],
    'points': [
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 0, 'nanosec': 0}
        },
        {
            'positions': [-0.025, 0.0, 0.82, -0.86, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 1, 'nanosec': 0}
        },
        {
            'positions': [-0.025, 0.1, 0.82, -0.86, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 2, 'nanosec': 0}
        },
        {
            'positions': [-0.025, 0.1, 0.82, -0.86, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 3, 'nanosec': 0}
        },
        {
            'positions': [-0.025, -0.44, 0.82, -0.86, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 4, 'nanosec': 0}
        },
        {
            'positions': [1.57, -0.05, 0.90, -0.71, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 5, 'nanosec': 0}
        },
        {
            'positions': [1.57, -0.05, 0.75, -0.81, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 6, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 7, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 9, 'nanosec': 0}
        }
    ]
}


def main(args=None): 
    rclpy.init(args=args) 

    # 创建一个FollowJointTrajectoryClient实例，指定控制器名称和ROS服务主题路径，用于与机器人控制器通信
    controller = FollowJointTrajectoryClient('abb_controller', '/abb/abb_joint_trajectory_controller')
    
    # 向控制器发送轨迹目标GOAL，其中10是超时时间（单位为秒）。
    controller.send_goal(GOAL, 10)
    
    # 启动一个循环，让节点保持活动状态，等待并处理ROS2通信事件，直到节点被显式关闭。
    rclpy.spin(controller)


if __name__ == '__main__':
    main()
