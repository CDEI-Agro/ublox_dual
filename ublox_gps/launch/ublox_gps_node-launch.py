# Copyright 2020 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the ublox gps node with c94-m8p configuration."""

import os

import ament_index_python.packages
import launch
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, RegisterEventHandler, EmitEvent
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config')
    
    params = os.path.join(config_directory, 'params.yaml')
    
    # NTRIP Client
    ntrip_ns = LaunchConfiguration('ntrip_ns')
    ntrip_launch = LaunchConfiguration('ntrip_launch')
    ntrip_ns_arg = DeclareLaunchArgument('ntrip_ns', default_value='ntrip', description='namespace of the NTRIP client package')
    ntrip_launch_arg = DeclareLaunchArgument('ntrip_launch', default_value='true', description='wether to launch the NTRIP client')
    ntrip = GroupAction(actions=[
        SetRemap(src = ('/', ntrip_ns, '/nmea'), dst = ('/ublox_base/nmea')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("ntrip_client"), 'ntrip_client_launch.py'])]),
            launch_arguments={'namespace': ntrip_ns,
                            'host': 'catnet-ip.icgc.cat',
                            'port': '2101',
                            'mountpoint': 'VRS3M',
                            'username': 'ignasiCDEI',
                            'password': 'cdei2023'}.items(),
            condition=IfCondition(ntrip_launch)
        )
    ])

    # Both ublox nodes
    base_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_base',
        output='both',
        parameters=[params],
        remappings=[('/rtcm', ('/', ntrip_ns, '/rtcm'))],
    )
    rover_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='gps',
        output='both',
        parameters=[params],
        remappings=[('/rtcm', ('/', ntrip_ns, '/rtcm/unused'))], # This a subscribed topic that anyone is going to publish. Rover does not need ntrip corrections. 
    )
    
    # Event handlers for On Exit
    base_on_exit = RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=base_node,
            on_exit=[EmitEvent(
                event=launch.events.Shutdown())],
        ))
    rover_on_exit = RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=rover_node,
            on_exit=[EmitEvent(
                event=launch.events.Shutdown())],
        ))
        
    # Launch all
    nodes = [
        ntrip_ns_arg,
        ntrip_launch_arg,
        ntrip,
        base_node,
        base_on_exit,
        rover_node,
        rover_on_exit,
    ]
    
    return launch.LaunchDescription(nodes)
