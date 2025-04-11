#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_namespace = LaunchConfiguration('robot_namespace', default='agent_0')
    TURTLEBOT3_MODEL = LaunchConfiguration('TURTLEBOT3_MODEL', default=TextSubstitution(text='burger_cam'))

    print("TURTLEBOT3MODEL")
    print(TURTLEBOT3_MODEL)

    if 'burger' in TURTLEBOT3_MODEL:
        TURTLEBOT3_MODEL_ = "burger"

    elif 'waffle' in TURTLEBOT3_MODEL:
        TURTLEBOT3_MODEL_ = "waffle"
    
    elif 'waffle_pi' in TURTLEBOT3_MODEL:
        TURTLEBOT3_MODEL_ = "waffle_pi"
    
    elif 'burger_cam' in TURTLEBOT3_MODEL:
        TURTLEBOT3_MODEL_ = "burger_cam"
    
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL_ + '.urdf'

    # print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('choirbot_examples'),
        'urdf',
        urdf_file_name)
    # urdf_path = PathJoinSubstitution([
    #     get_package_share_directory('choirbot_examples'),
    #     'urdf',
    #     PythonExpression(["'turtlebot3_' + '", TURTLEBOT3_MODEL, "' + '.urdf'"])
    # ])

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'TURTLEBOT3_MODEL',
            default_value='burger_cam',
            description='Choose model of the robot'),
            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_namespace,
            output='screen',
            remappings=remappings,
            parameters=[{
                'use_sim_time': use_sim_time,

                # 'robot_description': robot_desc,
                'robot_description': os.path.join(get_package_share_directory('choirbot_examples'), 'urdf',urdf_file_name ),
                # 'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
            }],
            # arguments=[
            #     '--ros-args',
            #     '--param', PythonExpression(["'robot_description:=' + '", urdf_path, "'"])
            # ]

        ),
    ])
