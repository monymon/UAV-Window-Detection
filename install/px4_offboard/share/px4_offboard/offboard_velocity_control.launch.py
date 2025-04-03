#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


TRUE = 't'
FALSE = 'f'


def generate_uav_nodes(context):
    mission_mode = LaunchConfiguration('mission_mode').perform(context)

    current_directory = os.path.dirname(__file__)
    mission_file_path = os.path.join(current_directory, 'mission.txt')
    mission_file = open(mission_file_path, "r")

    if mission_mode == TRUE:
        mission_steps = mission_file.readline()
    else:
        mission_steps = "0.0,0.0,0.0"

    return [
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='visualizer',
        #     name='visualizer'
        # ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='teleoperator',
            name='teleoperator',
            # prefix='gnome-terminal --'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='velocity_control',
            name='velocity',
            arguments=[mission_mode, mission_steps],
            # prefix='gnome-terminal --'
        )
    ]


def generate_launch_description():
    mission_mode_argument = DeclareLaunchArgument(
        'mission_mode',
        default_value=FALSE
    )

    return LaunchDescription([
        mission_mode_argument,
        OpaqueFunction(function=generate_uav_nodes)
    ])