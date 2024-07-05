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

"""Trajectory follower client for the Assembly robot used for multi-robot demonstration."""

import rclpy

from assembly_robot.asmb_robot_follow_joint_trajectory_client import FollowJointTrajectoryClient

#定义了一个字典GOAL，里面包含了机器人的关节轨迹信息。具体来说：
#'joint_names' 列出了参与轨迹控制的关节名称。
#'points' 是一系列轨迹点，每个点定义了各个关节在特定时间点的位置和到达该点所需的时间。
GOAL = {
    'joint_names': [
        'right_arm_joint1',
        'right_arm_joint2',
        'right_arm_joint3',
        'right_arm_joint4',
        'right_arm_joint5',
        'right_arm_joint6',
        'right_arm_joint7',
        'right_figner_joint2',
        'left_arm_joint1',
        'left_arm_joint2',
        'left_arm_joint3',
        'left_arm_joint4',
        'left_arm_joint5',
        'left_arm_joint6',
        'left_arm_joint7',
        'left_figner_joint2',
    ],
    'points': [
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 0, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 3, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 4, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 5, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 6, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.0, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 7, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 8, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 9, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 0, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 3, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 4, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 5, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 6, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.0, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 7, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 8, 'nanosec': 0}
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
    controller = FollowJointTrajectoryClient('asmbrobot_controller', '/asmbrobot/right_group_controller')
    controller.send_goal(GOAL, 10)
    
    rclpy.spin(controller)


if __name__ == '__main__':
    main()
