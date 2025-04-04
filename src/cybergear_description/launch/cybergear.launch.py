#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
def generate_launch_description():
    
    pkg = get_package_share_directory('cybergear_description')
    rviz_path = os.path.join(pkg, 'config', 'display.rviz')

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )
    
    # Xacro processing
    path_description = os.path.join(pkg, 'robot', 'visual', 'cybergear.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    # robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_xml, 'use_sim_time': False}]
    )

    # joint_state_publisher (no GUI)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': False}]
    )

    # Final LaunchDescription
    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])
