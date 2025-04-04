#!/usr/bin/env python3
#
# Authors:
#     Enrico Saccon     enrico.saccon [at] unitn.it
#     Placido Falqueto  placido.falqueto [at] unitn.it

import logging
import os
from pathlib import Path

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return


def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG

    map_env_pkg = get_package_share_directory("map_pkg")

    map_env_params_file_path = os.path.join(map_env_pkg, "config", "map_config.yaml")
    if not os.path.exists(map_env_params_file_path):
        raise Exception(
            "[{}] Map config file `{}` does not exist".format(
                __file__, map_env_params_file_path
            )
        )

    # General arguments
    map_env_params_file = LaunchConfiguration(
        "map_env_params_file", default=map_env_params_file_path
    )
    qt_qpa_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        DeclareLaunchArgument(
            "map_env_params_file",
            default_value=map_env_params_file_path,
            description="Full path to the map_pkg params file to use",
        ),   

    ]

    # List of nodes to launch
    nodes = [
        Node(
            package="map_pkg",
            executable="send_gates",
            name="send_gates",
            output="screen",
            parameters=[map_env_params_file],
        ),
        Node(
            package="map_pkg",
            executable="send_obstacles",
            name="send_obstacles",
            output="screen",
            parameters=[map_env_params_file],
        ),
        Node(
            package="map_pkg",
            executable="send_borders",
            name="send_borders",
            output="screen",
            parameters=[map_env_params_file],
        ),
        Node(
            package="map_pkg",
            executable="send_victims",
            name="send_victims",
            output="screen",
            parameters=[map_env_params_file],
        ),
    ]

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()
    # Add the action to set the environment variable
    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=print_env))

    for node in nodes:
        ld.add_action(node)

    return ld
