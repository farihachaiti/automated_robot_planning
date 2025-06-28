import math
import numpy as np
from scipy.interpolate import CubicSpline
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import logging

class DubinsPath():
    def __init__(self, start, end, curvature, robot_positions, logger):
        # Define parameters
        self.turning_angle = math.pi/3
        self.min_turning_radius = 1.0/curvature
        self.logger = logger

        self.start = start
        self.end_point = end
        self.distance = self.get_distance(self.start, self.end_point)
        self.initial_heading = self.get_initial_heading(self.start, self.end_point)
        self.theta_diff = self.get_theta_diff(self.start, self.end_point)
        self.sum_arcs = 2 * math.pi * self.min_turning_radius
        self.robot_positions = robot_positions
        
        self.yaml_path = os.path.join(
        get_package_share_directory('map_pkg'),
        'config',
        'full_config.yaml'
        )

    def get_distance(self, start, end):
        return math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)

    def get_initial_heading(self, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        return math.atan2(dy, dx)

    def get_theta_diff(self, start, end):
        theta_diff = end[2] - start[2]
        return (theta_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize angle to [-pi, pi]

    def compute_segments(self, initial_heading, distance, start):
        end_point = self.end_point
        sequence = None
        
        # All angles should be in radians
        initial_heading_rad = initial_heading
        theta_diff_rad = self.theta_diff
        
        # Dubins path selection conditions (in order of preference)
        if distance >= 2 * self.min_turning_radius:
            # Check for CSC paths first (more optimal when possible)
            if initial_heading_rad >= 0 and (initial_heading_rad + theta_diff_rad) >= 0:
                end_point, length, sequence = self.path_segments(initial_heading_rad, distance, ['L', 'S', 'L'], start)
            elif initial_heading_rad < 0 and (initial_heading_rad + theta_diff_rad) < 0:
                end_point, length, sequence = self.path_segments(initial_heading_rad, distance, ['R', 'S', 'R'], start)
            elif (initial_heading_rad + theta_diff_rad) >= 0:
                end_point, length, sequence = self.path_segments(initial_heading_rad, distance, ['L', 'S', 'R'], start)
            else:
                end_point, length, sequence = self.path_segments(initial_heading_rad, distance, ['R', 'S', 'L'], start)
        else:
            # Check for CCC paths when distance is small
            if initial_heading_rad >= 0:
                end_point, length, sequence = self.path_segments(initial_heading_rad, distance, ['L', 'R', 'L'], start)
            else:
                end_point, length, sequence = self.path_segments(initial_heading_rad, distance, ['R', 'L', 'R'], start)
        
        # Fallback to straight line if very close
        if sequence is None or distance < self.min_turning_radius:
            end_point, length, sequence = self.path_segments(initial_heading_rad, distance, ['S'], start)
        
        return (start, end_point), sequence, length

    def path_segments(self, initial_heading, distance, segment_type, start):
        current_point = np.array([start[0], start[1], start[2]])
        total_length = 0.0
        
        for s in segment_type:
            if s == 'L':
                # Left turn
                center_x = current_point[0] + self.min_turning_radius * math.sin(initial_heading)
                center_y = current_point[1] - self.min_turning_radius * math.cos(initial_heading)
                
                delta_theta = self.turning_angle
                arc_length = delta_theta * self.min_turning_radius
                
                end_x = center_x - self.min_turning_radius * math.sin(initial_heading + delta_theta)
                end_y = center_y + self.min_turning_radius * math.cos(initial_heading + delta_theta)
                end_theta = (current_point[2] + delta_theta) % (2 * math.pi)
                
                current_point = np.array([end_x, end_y, end_theta])
                initial_heading += delta_theta
                total_length += arc_length
                
            elif s == 'R':
                # Right turn
                center_x = current_point[0] - self.min_turning_radius * math.sin(initial_heading)
                center_y = current_point[1] + self.min_turning_radius * math.cos(initial_heading)
                
                delta_theta = -self.turning_angle
                arc_length = abs(delta_theta) * self.min_turning_radius
                
                end_x = center_x - self.min_turning_radius * math.sin(initial_heading + delta_theta)
                end_y = center_y + self.min_turning_radius * math.cos(initial_heading + delta_theta)
                end_theta = (current_point[2] + delta_theta) % (2 * math.pi)
                
                current_point = np.array([end_x, end_y, end_theta])
                initial_heading += delta_theta
                total_length += arc_length
                
            elif s == 'S':
                # Straight segment
                end_x = current_point[0] + distance * math.cos(initial_heading)
                end_y = current_point[1] + distance * math.sin(initial_heading)
                end_theta = current_point[2]
                
                current_point = np.array([end_x, end_y, end_theta])
                total_length += distance
        
        return tuple(current_point), total_length, ''.join(segment_type)

    def close_enough(self, config1, config2, threshold=0.05):
        position_diff = math.sqrt((config1[0]-config2[0])**2 + (config1[1]-config2[1])**2)
        angle_diff = min(abs(config1[2]-config2[2]), 2*math.pi - abs(config1[2]-config2[2]))
        return position_diff <= threshold and angle_diff <= math.radians(5)

    def plan_path(self, start, goal):
        path = [start]
        current_pos = start
        self.obstacles, self.map_bounds = self.load_obstacles_from_yaml(self.yaml_path)
        max_steps = 200  # Safety limit
        steps = 0
        robot_radius = 0.4
        min_robot_distance=0.05
        min_obstacle_distance=0.05
        last_distance = self.get_distance(current_pos, goal)
        while not self.close_enough(current_pos, goal):
            # Update path parameters
            self.distance = self.get_distance(current_pos, goal)
            self.initial_heading = self.get_initial_heading(current_pos, goal)
            self.theta_diff = self.get_theta_diff(current_pos, goal)

            # Compute next segment
            (_, new_pos), sequence, length = self.compute_segments(
                self.initial_heading,
                min(self.distance, 2*self.min_turning_radius),
                current_pos
            )
            #self.logger.error(f"The new position is: {new_pos}")
            valid_new_pos = self.find_valid_position_toward_goal(
                new_pos, goal, self.map_bounds, self.robot_positions, self.obstacles, robot_radius=0.4,
                min_robot_distance=0.05, min_obstacle_distance=0.05, steps=10
            )
            #self.logger.error(f"The valid new position is: {valid_new_pos}")
            if valid_new_pos is None:
                logging.warning("Could not find valid next step, path planning failed.")
                break
            # Avoid repeated points
            #if self.close_enough(valid_new_pos, current_pos, threshold=1e-6):
            #    logging.warning("No progress made, path planning stuck.")
            #    break
            if self.close_enough(new_pos, valid_new_pos, threshold=0.5) and valid_new_pos==goal:
                path.append(valid_new_pos)
                return path
            # Interpolate between current_pos and valid_new_pos
            intermediate_points = self.interpolate_points(current_pos, valid_new_pos, step_size=0.2)
            for pt in intermediate_points:
                # Use the same check_valid logic as in find_valid_position_toward_goal
                def check_valid(x, y):
                    x_min, x_max, y_min, y_max = self.map_bounds
                    if not (x_min + robot_radius <= x <= x_max - robot_radius and y_min + robot_radius <= y <= y_max - robot_radius):
                        return False

                    # Check distance from other robots
                    for rx, ry, _ in self.robot_positions:
                        dist = ((x - rx)**2 + (y - ry)**2)**0.5
                        if dist < (2 * robot_radius + min_robot_distance):
                            return False

                    # Check distance from obstacles
                    for ox, oy, radius in self.obstacles:
                        dist = ((x - ox)**2 + (y - oy)**2)**0.5
                        if dist < (radius + robot_radius + min_obstacle_distance):
                            return False
                    return True
                if check_valid(pt[0], pt[1]):
                    path.append(pt)
                else:
                    break
            current_pos = pt
    
            steps += 1
            if steps >= max_steps:
                logging.warning("Maximum steps reached, stopping path planning.")
                break
        
        return path

    def make_close_enough(self, start, end, length=None):
        t = 10
        while not self.close_enough(self.end_point, end, 0.05):
            final_end_point = self.interpolate_to_target_point(end, self.end_point)
            t-=1
            end = final_end_point
            if t==0:
                break
        if not self.close_enough(end, self.end_point, 0.05):
            if t==0:
                return tuple(end)
            return self.make_close_enough(start, end, length)
        else:
            return tuple(end)

    def interpolate_to_target_point(self, last_point, target_point):
        """
        Interpolate one point along a Dubins path to the target point using cubic spline interpolation.
        """
        # Define the parameter space
        t = np.array([0, 1])  # Two control points
        
        # Ensure last_point and target_point are 1D arrays
        last_point = np.array(last_point)
        target_point = np.array(target_point)
        
        # Ensure dubins_points is a 2D array with two rows
        
        dubins_points = np.vstack((last_point, target_point))

        spline = CubicSpline(t, dubins_points)
        
        # Evaluate the spline at t=0.5 to get the interpolated point
        interpolated_point = spline(0.05)
        
        return tuple(interpolated_point)


    def load_obstacles_from_yaml(self, yaml_path):
        """Load obstacle data and map dimensions from YAML configuration file.
        
        Returns:
            tuple: (obstacles, map_bounds) where map_bounds is (x_min, x_max, y_min, y_max)
        """
        obstacles = []
        map_bounds = (-6.0, 6.0, -6.0, 6.0)  # Default bounds if not specified in YAML
        x_min, x_max, y_min, y_max = map_bounds

      
        
        try:
            with open(self.yaml_path, 'r') as file:
                config = yaml.safe_load(file)
                
            # Get map dimensions from root parameters
            root_params = config.get('/', {}).get('ros__parameters', {})
            dx = float(root_params.get('dx', 12.0))  # Default to 12.0 if not specified
            dy = float(root_params.get('dy', 22.0))  # Default to 12.0 if not specified
            
            # Calculate map bounds (centered at origin)
            map_bounds = (-dx/2, dx/2, -dy/2, dy/2)
            
            # Get obstacle parameters
            obs_params = config.get('/send_obstacles', {}).get('ros__parameters', {})
            
            # Process vector-based obstacles
            vect_x = obs_params.get('vect_x', [-3.7217116794479477, 1.0015742916634345, -0.09764584264339327, 0.39021553699774003, -3.7347266625811617])
            vect_y = obs_params.get('vect_y', [-6.5895867813851305, -5.990009494173584, -6.081339720357923, 1.7851924412359708, 2.062924792806049])
            vect_dim_x = obs_params.get('vect_dim_x', [0.9472303576889589, 0.854017782573675, 0.6822544643904452, 0.9473346271529302, 0.8065071256362644])
            vect_dim_y = obs_params.get('vect_dim_y', [0.6449529454098106, 0.9815630335593322, 0.5073320193413408, 0.8686996689784644, 0.678918639515615])
            print(f'vect_x: {vect_x}')
            print(f'vect_y: {vect_y}')
            print(f'vect_dim_x: {vect_dim_x}')
            print(f'vect_dim_y: {vect_dim_y}')
            
            # Add vector-based obstacles
            for i in range(min(len(vect_x), len(vect_y))):
                x = vect_x[i]
                y = vect_y[i]
                # Use max dimension as radius for simplicity
                radius = max(
                    vect_dim_x[i] if i < len(vect_dim_x) else 0.5,
                    vect_dim_y[i] if i < len(vect_dim_y) else 0.5
                ) / 2.0
                obstacles.append((x, y, radius))
            
            logging.info(f"Loaded {len(obstacles)} obstacles from YAML config")
            
        except Exception as e:
            logging.error(f"Error loading obstacles from YAML: {e}")
            # Add some default obstacles if loading fails
            obstacles = [(0.0, 0.0, 1.0)]
        
        return obstacles, map_bounds



    def find_valid_position_toward_goal(self, new_pos, goal_pos, map_bounds, robot_positions, obstacles, robot_radius=0.4, min_robot_distance=0.05, min_obstacle_distance=0.05, steps=10):
        """
        Check if new_pos is valid. If not, interpolate from new_pos toward goal_pos in steps,
        returning the first valid position found, or None if none are valid.
        """
        def check_valid(x, y):
            x_min, x_max, y_min, y_max = map_bounds
            if not (x_min + robot_radius <= x <= x_max - robot_radius and y_min + robot_radius <= y <= y_max - robot_radius):
                return False

            # Check distance from other robots
            for rx, ry, _ in robot_positions:
                dist = ((x - rx)**2 + (y - ry)**2)**0.5
                if dist < (2 * robot_radius + min_robot_distance):
                    return False

            # Check distance from obstacles
            for ox, oy, radius in obstacles:
                dist = ((x - ox)**2 + (y - oy)**2)**0.5
                if dist < (radius + robot_radius + min_obstacle_distance):
                    return False
            return True

        x, y, yaw = new_pos[0], new_pos[1], new_pos[2]
        gx, gy = goal_pos[0], goal_pos[1]
        if check_valid(x, y):
            return (x, y, yaw)
        else:            
            intermediate_points = self.interpolate_points(new_pos, goal_pos, step_size=0.5)
            for pt in intermediate_points:
                if check_valid(pt[0], pt[1]):  # You may need to implement this check
                    break
            return pt              
               
        return None  # No valid position found

    def interpolate_points(self, start, end, step_size=0.5):
        x0, y0, theta0 = start
        x1, y1, theta1 = end
        # For two points, t = [0, 1]
        t = np.array([0, 1])
        x_points = np.array([x0, x1])
        y_points = np.array([y0, y1])
        theta_points = np.array([theta0, theta1])

        # Create cubic splines for x and y
        cs_x = CubicSpline(t, x_points)
        cs_y = CubicSpline(t, y_points)
        # For theta, linear interpolation is usually sufficient
        def interp_theta(t_val):
            return theta0 + (theta1 - theta0) * t_val

        # Number of steps
        distance = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        steps = max(1, int(distance / step_size))
        points = []
        for i in range(1, steps + 1):
            t_val = i / steps
            x = cs_x(t_val)
            y = cs_y(t_val)
            theta = interp_theta(t_val)
            points.append((x, y, theta))
        return points



def main():
    start = (-1.31, -0.16, 1.01)  # Fixed typo in y value
    goal = (-0.9817254262488451, -8.060254037844386, math.pi/2)
    
    dubinspath = DubinsPath(start, goal, 1.0, [], logging.getLogger(__name__))  # Curvature = 1.0
    
    path = dubinspath.plan_path(start, goal)
    
    print("Path points:")
    for i, point in enumerate(path):
        print(f"Point {i}: x={point[0]:.2f}, y={point[1]:.2f}, θ={math.degrees(point[2]):.2f}°")
    
    print(f"\nTotal path length: {len(path)} segments")
#if __name__ == '__main__':
#    main()