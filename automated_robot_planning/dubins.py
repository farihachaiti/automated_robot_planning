import math
import numpy as np
from scipy.interpolate import CubicSpline
#from ament_index_python.packages import get_package_share_directory
import os
import yaml
import logging
import matplotlib.pyplot as plt
import tf_transformations

class DubinsPath():
    def __init__(self, start, end, curvature=None, robot_positions=None, map_bounds=None, obstacles=None, logger=None):
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
        self.map_bounds = map_bounds
        self.obstacles = obstacles
        
        self.yaml_path = os.path.join(
        #get_package_share_directory('map_pkg'),
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

    def dubins_segment_lengths(self, alpha, beta, d):
        # Returns a dict of {sequence: (t, p, q)} for all Dubins path types
        # alpha, beta: angles in radians, d: normalized distance
        paths = {}
        def mod2pi(theta):
            return theta % (2 * math.pi)

        # LSL
        tmp0 = d + math.sin(alpha) - math.sin(beta)
        p_squared = 2 + d**2 - 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha)-math.sin(beta))
        if p_squared >= 0:
            tmp1 = math.atan2((math.cos(beta)-math.cos(alpha)), tmp0)
            t = mod2pi(-alpha + tmp1)
            p = math.sqrt(p_squared)
            q = mod2pi(beta - tmp1)
            paths['LSL'] = (t, p, q)

        # RSR
        tmp0 = d - math.sin(alpha) + math.sin(beta)
        p_squared = 2 + d**2 - 2*math.cos(alpha-beta) + 2*d*(-math.sin(alpha)+math.sin(beta))
        if p_squared >= 0:
            tmp1 = math.atan2((math.cos(alpha)-math.cos(beta)), tmp0)
            t = mod2pi(alpha - tmp1)
            p = math.sqrt(p_squared)
            q = mod2pi(-beta + tmp1)
            paths['RSR'] = (t, p, q)

        # LSR
        p_squared = -2 + d**2 + 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha)+math.sin(beta))
        if p_squared >= 0:
            p = math.sqrt(p_squared)
            tmp2 = math.atan2((-math.cos(alpha)-math.cos(beta)), (d+math.sin(alpha)+math.sin(beta)))
            t = mod2pi(-alpha + tmp2)
            q = mod2pi(-mod2pi(beta) + tmp2)
            paths['LSR'] = (t, p, q)

        # RSL
        p_squared = -2 + d**2 + 2*math.cos(alpha-beta) - 2*d*(math.sin(alpha)+math.sin(beta))
        if p_squared >= 0:
            p = math.sqrt(p_squared)
            tmp2 = math.atan2((math.cos(alpha)+math.cos(beta)), (d-math.sin(alpha)-math.sin(beta)))
            t = mod2pi(alpha - tmp2)
            q = mod2pi(beta - tmp2)
            paths['RSL'] = (t, p, q)

        # RLR
        tmp_rlr = (6. - d**2 + 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha)-math.sin(beta))) / 8.
        if abs(tmp_rlr) <= 1:
            p = mod2pi(2*math.pi - math.acos(tmp_rlr))
            t = mod2pi(alpha - math.atan2((math.cos(alpha)-math.cos(beta)), d-math.sin(alpha)+math.sin(beta)) + p/2.)
            q = mod2pi(alpha - beta - t + p)
            paths['RLR'] = (t, p, q)

        # LRL
        tmp_lrl = (6. - d**2 + 2*math.cos(alpha-beta) + 2*d*(-math.sin(alpha)+math.sin(beta))) / 8.
        if abs(tmp_lrl) <= 1:
            p = mod2pi(2*math.pi - math.acos(tmp_lrl))
            t = mod2pi(-alpha + math.atan2((math.cos(alpha)-math.cos(beta)), d+math.sin(alpha)-math.sin(beta)) + p/2.)
            q = mod2pi(mod2pi(beta) - alpha - t + p)
            paths['LRL'] = (t, p, q)

        # Fallback: straight segment (S) if start and goal are aligned
        if d > 0:
            # Only add if the heading difference is small (i.e., almost aligned)
            if abs((alpha - beta) % (2 * math.pi)) < 0.05:
                t = 0.0
                p = d
                q = 0.0
                paths['S'] = (t, p, q)
        return paths

    def compute_segments(self, initial_heading, distance, start):
        # True Dubins path equations implementation
        x0, y0, th0 = start
        x1, y1, th1 = self.end_point
        dx = x1 - x0
        dy = y1 - y0
        D = math.hypot(dx, dy)
        d = D / self.min_turning_radius
        theta = math.atan2(dy, dx)
        alpha = (initial_heading - theta) % (2*math.pi)
        beta = (th1 - theta) % (2*math.pi)
        # Compute all Dubins paths
        all_paths = self.dubins_segment_lengths(alpha, beta, d)
        best = None
        best_length = float('inf')
        best_seq = None
        best_segs = None
        for seq, (t, p, q) in all_paths.items():
            total = (t + p + q) * self.min_turning_radius
            penalty = 0.0
            if seq in ['LRL', 'RLR']:
                penalty += 10.0  # Large penalty for CCC paths (sharp turns)
            if seq == 'S':
                segs = [p * self.min_turning_radius]  # Only one segment for straight
            else:
                segs = [t * self.min_turning_radius, p * self.min_turning_radius, q * self.min_turning_radius]
                if seq == 'S' and p * self.min_turning_radius < 5.0:  # Short straight segment
                    penalty += 5.0
            if total + penalty < best_length:
                best_length = total + penalty
                best_seq = list(seq)
                best_segs = segs
        if best_seq is None:
            # fallback to straight
            best_seq = ['S']
            best_segs = [distance]
        end_point, length, sequence_str = self.path_segments(initial_heading, best_segs, best_seq, start)
        return (start, end_point), sequence_str, length

    def path_segments(self, initial_heading, segment_lengths, segment_type, start):
        """
        Compute the final pose and total length after following a sequence of Dubins path segments.

        Args:
            initial_heading (float): Starting orientation (radians).
            segment_lengths (list of float): Lengths for each segment.
            segment_type (list of str): Sequence of 'L', 'R', 'S'.
            start (list or np.array): Starting pose [x, y, theta].

        Returns:
            tuple: (final_pose, total_length, segment_type_str)
        """
        current_point = np.array([start[0], start[1], start[2]])
        total_length = 0.0

        for s, seg_len in zip(segment_type, segment_lengths):
            if s == 'L':
                # Left turn
                center_x = current_point[0] + self.min_turning_radius * math.sin(initial_heading)
                center_y = current_point[1] - self.min_turning_radius * math.cos(initial_heading)
                delta_theta = seg_len / self.min_turning_radius
                arc_length = abs(delta_theta) * self.min_turning_radius

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
                delta_theta = -seg_len / self.min_turning_radius
                arc_length = abs(delta_theta) * self.min_turning_radius

                end_x = center_x - self.min_turning_radius * math.sin(initial_heading + delta_theta)
                end_y = center_y + self.min_turning_radius * math.cos(initial_heading + delta_theta)
                end_theta = (current_point[2] + delta_theta) % (2 * math.pi)

                current_point = np.array([end_x, end_y, end_theta])
                initial_heading += delta_theta
                total_length += arc_length

            elif s == 'S':
                # Straight segment
                end_x = current_point[0] + seg_len * math.cos(initial_heading)
                end_y = current_point[1] + seg_len * math.sin(initial_heading)
                end_theta = current_point[2]

                current_point = np.array([end_x, end_y, end_theta])
                total_length += seg_len

        return tuple(current_point), total_length, ''.join(segment_type)

    def close_enough(self, config1, config2, threshold=0.05):
        #print(config1)
        #print(config2)
        position_diff = math.sqrt((config1[0]-config2[0])**2 + (config1[1]-config2[1])**2)
        
        angle_diff = min(abs(config1[2]-config2[2]), 2*math.pi - abs(config1[2]-config2[2]))
        return position_diff <= threshold and angle_diff <= math.radians(threshold*100)

    def plan_path(self, start, goal):
        path = [start]
        current_pos = start
        
        max_steps = 200  # Safety limit
        steps = 0
        robot_radius = 0.4
        min_robot_distance = 0.05
        min_obstacle_distance = 0.05
        
      
        # Use a reasonable threshold for convergence
        threshold = 50.05
        step_size = 2 * self.min_turning_radius
        min_step_size = 0.5  # You can adjust this value

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

        print(f"Starting path planning from {start} to {goal}")
        
        while not self.close_enough(current_pos, goal, 0.05):
            # Update path parameters
            last_distance = self.get_distance(current_pos, goal)
            self.initial_heading = self.get_initial_heading(current_pos, goal)
            self.theta_diff = self.get_theta_diff(current_pos, goal)
            step = max(min(self.distance, step_size), min_step_size)
            # Compute next segment
            (_, new_pos), sequence, length = self.compute_segments(
                self.initial_heading,
                step,
                current_pos
            )

            print(f"Step {steps}:")
            print(f"  Current position: {current_pos}")
            print(f"  Next position: {new_pos}")
            print(f"  Sequence: {sequence}, Segment length: {length:.3f}")
            print(f"  Distance to goal: {self.get_distance(new_pos, goal):.3f}")

            if new_pos is None:
                print("Could not find valid next step, path planning failed.")
                break
            # Avoid repeated points
            if self.close_enough(current_pos, new_pos, threshold=0.05):
                print("No progress made, path planning stuck.")
                break
    
            #if check_valid(new_pos[0], new_pos[1]):
            #    path.append(new_pos)
            #    current_pos = new_pos
            #else: 
            #    current_pos = new_pos
            #    continue 
       
            path.append(new_pos)
            current_pos = new_pos
    
            steps += 1
            if steps >= max_steps:
                print("Maximum steps reached, stopping path planning.")
                break
        print(self.close_enough(current_pos, goal, 0.05))
        print(f"Path planning finished after {steps} steps. Path length: {len(path)}")
        if len(path) < 2:
            smoothed_path = path
        else:
            smoothed_path = self.smooth_path(path)
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

    def smooth_path(self, path, num_points=100):
        if len(path) < 2:
            return path  # Not enough points to smooth
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ts = np.linspace(0, 1, len(xs))
        t_fine = np.linspace(0, 1, num_points)
        cs_x = CubicSpline(ts, xs)
        cs_y = CubicSpline(ts, ys)
        xs_smooth = cs_x(t_fine)
        ys_smooth = cs_y(t_fine)
        return list(zip(xs_smooth, ys_smooth))

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


def load_obstacles_from_yaml():
    obstacles = []
    map_bounds = []
    # Add some default obstacles if loading fails
    vect_dim_x = [0.8011211001104662, 0.5121030174270204]
    vect_dim_y = [0.7663025495066591, 1.0]
    vect_x = [2.3572118362481698, -1.3509232375009814]
    vect_y = [3.4168598650484014, 5.355224601477801]
    dx = 10.0
    dy = 20.0
    map_bounds = (-dx/2, dx/2, -dy/2, dy/2)
    for i in range(min(len(vect_x), len(vect_y))):
        x = vect_x[i]
        y = vect_y[i]
        # Use max dimension as radius for simplicity
        radius = max(
            vect_dim_x[i] if i < len(vect_dim_x) else 0.5,
            vect_dim_y[i] if i < len(vect_dim_y) else 0.5
        ) / 2.0
    return obstacles, map_bounds

    




def main():
    start = (-3.31, -5.42, -0.48)  # Fixed typo in y value
    start2 = (3.14, 1.42, 1.71)
    goal2 = (1.06, 8.06, math.pi/2)
    goal = (-3.086493, -8.060254, -1.570796)
    start3 = (-3.530316241159565, 3.164402679769098, -2.4564637764115815)
    goal3 = (-6.44371743686115, -4.9596620854277615, -2.6179938779914944)
    #obstacles, map_bounds = load_obstacles_from_yaml()
    path_planner = PathPlanner(start, goal)
    path = path_planner.robot_task(start)
    #dubinspath = DubinsPath(start, goal, 2.0, [(-3.31, -5.42, -0.48), (-1.64, -7.46, -2.79), (1.87, -1.77, -1.41)], map_bounds, obstacles, logging.getLogger(__name__))  # Curvature = 1.0
   
    #path = dubinspath.plan_path(start, goal)
    
    print("Path points:")
    for i, point in enumerate(path):
        print(f"Point {i}: x={point[0]:.2f}, y={point[1]:.2f}, θ={math.degrees(point[2]):.2f}°")
    
    print(f"\nTotal path length: {len(path)} segments")

    # Visualization
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    plt.figure(figsize=(8, 8))
    plt.plot(xs, ys, 'bo-', label='Dubins Path')
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')
    plt.title('Dubins Path Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
   main()