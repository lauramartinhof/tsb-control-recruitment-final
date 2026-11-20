# Copyright 2026 Laura Martinho
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

"""Launch file for TSB sensor, PID, allocation and path following nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with simulation time enabled."""
    pkg_share = get_package_share_directory('tsb_control')
    config_file = os.path.join(pkg_share, 'config', 'pid_params.yaml')

    sensor_node = Node(
        package='tsb_control',
        executable='sensor_node',
        name='sensor_processing_node',
        parameters=[config_file],
        output='screen',
    )

    pid_node = Node(
        package='tsb_control',
        executable='pid_node',
        name='pid_node',
        parameters=[config_file],
        output='screen',
    )

    allocation_node = Node(
        package='tsb_control',
        executable='allocation_node',
        name='allocation_node',
        parameters=[config_file],
        output='screen',
    )

    path_follower_node = Node(
        package='tsb_control',
        executable='path_follower_node',
        name='path_follower_node',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        sensor_node,
        pid_node,
        allocation_node,
        path_follower_node,
    ])
