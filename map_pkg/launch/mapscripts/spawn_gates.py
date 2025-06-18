#!/usr/bin/env python3
import os, yaml, random
import numpy as np
import time

from launch.substitutions import PythonExpression
from launch_ros.actions import Node

from geo_utility import *
from spawn_borders import get_borders_points


L = 1.0 # Default size for gate
DELTA = L/2.0 + 0.1 # 0.1 is half the border width + something


def spawn_gates(context):
    gen_map_params_file = context.launch_configurations['gen_map_params_file']
    with open(gen_map_params_file, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        gates = params["/**/send_gates"]['ros__parameters']

    gate_model_path = os.path.join(context.launch_configurations['elements_models_path'], 'gate', 'model.sdf')
    
    # Check that all vectors have the same number of gates
    assert len(gates['x']) == len(gates['y']) == len(gates['yaw']), "The number of gates in the conf file is wrong. The script for generating the configuration made a mistake."

    # Generate unique timestamp for this spawn session
    timestamp = int(time.time() * 1000)

    nodes = []
    for gate in range(len(gates['x'])):
        print(f"Spawning gate in {gates['x'][gate]}, {gates['y'][gate]} with yaw {gates['yaw'][gate]}")
        # If x and y are 0, then the gate is random
        yaw = gates['yaw'][gate]
        gate_polygon = square(gates['x'][gate], gates['y'][gate], L, gates['yaw'][gate])

        center = gate_polygon.centroid.coords[0]
        
        # Create unique entity name with timestamp
        entity_name = f"gate_{timestamp}_{gate}"
        
        nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', PythonExpression(["'", gate_model_path, "'"]),
                    '-entity', PythonExpression(["'", entity_name, "'"]),
                    '-x', PythonExpression(["'", str(center[0]), "'"]),
                    '-y', PythonExpression(["'", str(center[1]), "'"]),
                    '-z', PythonExpression(["'", str(0.0001), "'"]),
                    '-Y', PythonExpression(["'", str(yaw), "'"]),

                ],
            )
        )

    return nodes
