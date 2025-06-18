import os, yaml, random
import numpy as np
from pathlib import Path
import time

from launch.substitutions import PythonExpression
from launch_ros.actions import Node

from geo_utility import *
from spawn_borders import get_borders_points


def spawn_obstacles(context):
    gen_map_params_file = context.launch_configurations['gen_map_params_file']
    with open(gen_map_params_file, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        obstacles = params["/**/send_obstacles"]['ros__parameters']

    box_model_path = os.path.join(context.launch_configurations['elements_models_path'], 'box', 'model.sdf')
    cylinder_model_path = os.path.join(context.launch_configurations['elements_models_path'], 'cylinder', 'model.sdf')
    
    # Check that all vectors have the same number of obstacles
    assert len(obstacles['vect_x']) == len(obstacles['vect_y']) == \
        len(obstacles['vect_yaw']) == len(obstacles["vect_type"]) == \
        len(obstacles['vect_dim_x']) == len(obstacles["vect_dim_y"]), \
        "The number of obstacles in the conf file is wrong. The script for generating the configuration made a mistake."

    # Generate unique timestamp for this spawn session
    timestamp = int(time.time() * 1000)
    
    nodes = []
    for obs in range(len(obstacles['vect_x'])):
        center = (obstacles['vect_x'][obs], obstacles['vect_y'][obs])
        
        if obstacles["vect_type"][obs] == "cylinder":
            with open(cylinder_model_path, 'r') as file:
                obstacle_model = file.read()
            obstacle_model = obstacle_model.replace("##radius##", str(obstacles['vect_dim_x'][obs]/2))
            obstacle_model = obstacle_model.replace("##height##", str(obstacles['vect_dim_y'][obs]))
        else:
            with open(box_model_path, 'r') as file:
                obstacle_model = file.read()
            obstacle_model = obstacle_model.replace("##dx##", str(obstacles['vect_dim_x'][obs]))
            obstacle_model = obstacle_model.replace("##dy##", str(obstacles['vect_dim_y'][obs]))
            obstacle_model = obstacle_model.replace("##dz##", str(obstacles['vect_dim_y'][obs]))

        with open('tmp.sdf', 'w') as file:
            file.write(obstacle_model)
            path = Path(file.name).resolve()

        # Create unique entity name with timestamp and obstacle index
        entity_name = f"{obstacles['vect_type'][obs]}_{timestamp}_{obs}"
        

        nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                            '-file', PythonExpression(["'", str(path), "'"]),
                            '-entity', PythonExpression(["'", entity_name, "'"]),
                            '-x', PythonExpression(["'", str(center[0]), "'"]),
                            '-y', PythonExpression(["'", str(center[1]), "'"]),
                            '-z', PythonExpression(["'", str(0.0001), "'"]),
                            '-Y', PythonExpression(["'", str(obstacles["vect_yaw"][obs]), "'"]),
                            '--allow_renaming',
                    
                ],
            )
        )

    return nodes


