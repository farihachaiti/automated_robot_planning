import math
import csv
import numpy as np
from scipy.interpolate import CubicSpline


class DubinsPath():
    def __init__(self, start, end, curvature):
    # Define parameters
        self.turning_angle = math.pi/3
        self.min_turning_radius = 1/curvature

        self.start = start
        self.end_point = end
        self.distance = self.get_distance(self.start, self.end_point)
        self.initial_heading = self.get_initial_heading(self.start, self.end_point)
        self.theta_diff = self.get_theta_diff(self.start, self.end_point)
        self.sum_arcs = 2 * math.pi * self.min_turning_radius
        self.theta_diff = self.get_theta_diff(self.start, self.end_point) 


    def get_distance(self, start, end):
        return math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)


    def get_initial_heading(self, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        return math.atan2(dy, dx)



    def recur_lens(self, lens, segs_=None, sub=False):
        if segs_:
            segs = segs_
        else:
            segs = []
        sub = [lens[i] < 50.0 if lens[i] < 50.0 else False for i in range(len(lens))]

        if all(sub):
            segs.append(lens)
        else:
            for i in lens:
                if i>=50.0:
                    if not (0.0<=i%2<=1.0 or 0.0<=i%3<=1.0 or 0.0<=i%4<=1.0 or 0.0<=i%5<=1.0 or 0.0<=i%6<=1.0 or 0.0<=i%7<=1.0):
                        segs.append(i)
                    else:
                        segment_lengths, num_of_segs = self.get_segment_lengths(i)
                        result = [None if not (0.0<=x%2<=1.0 or 0.0<=x%3<=1.0 or 0.0<=x%4<=1.0 or 0.0<=x%5<=1.0 or 0.0<=x%6<=1.0 or 0.0<=x%7<=1.0) else x for x in segment_lengths]
                        if all(x is None for x in result):
                            segs.append(segment_lengths)
                        else:                      
                            sg = self.recur_lens(segment_lengths, False, True)
                            if sg:
                                segs.append(self.flatten(sg))
                        
                else:
                    segs.append(i)
        
        return segs



    def flatten(self, lst):
        flat_list = []
        for item in lst:
            if isinstance(item, list):
                flat_list.extend(self.flatten(item))
            else:
                flat_list.append(item)
        return flat_list
    


    def get_segment_lengths(self, x):
        i = x - 1
        f = []
        sum = 0
        if x>=50.0:
            while 0.0<=x%2<=1.0 or 0.0<=x%3<=1.0 or 0.0<=x%4<=1.0 or 0.0<=x%5<=1.0 or 0.0<=x%6<=1.0 or 0.0<=x%7<=1.0:
                if 0.0<=x%2<=1.0:
                    i = x/2
                elif 0.0<=x%3<=1.0:
                    i = x/3
                elif 0.0<=x%4<=1.0:
                    i = x/4
                elif 0.0<=x%5<=1.0:
                    i = x/5
                elif 0.0<=x%6<=1.0:
                    i = x/6
                elif 0.0<=x%7<=1.0:
                    i = x/7
                
                f.append(i)
                sum = sum + i
                if not (0.0<=x%2<=1.0 or 0.0<=x%3<=1.0 or 0.0<=x%4<=1.0 or 0.0<=x%5<=1.0 or 0.0<=x%6<=1.0 or 0.0<=x%7<=1.0):
                    k = x - sum
                    f.append(k)
                    break
                x = i
                
            if not (0.0<=x%2<=1.0 or 0.0<=x%3<=1.0 or 0.0<=x%4<=1.0 or 0.0<=x%5<=1.0 or 0.0<=x%6<=1.0 or 0.0<=x%7<=1.0):
                f.append(x)
                return f, len(f)
            if f:
                return list(np.sort(f)), len(f)
            else:
                return False
        else:
            return [x], int(1)
    


    def segmentize_path(self, start, distance, num_segments, sub_segment_lengths, paths, end):
        # Generate sub-segments
        #if isinstance(num_segments, list):
        
        for j in num_segments:
            for i in range(j):
                # Length of the current sub-segment
                
                if isinstance(sub_segment_lengths[i], list):
                    paths.append(self.compute_segments(self.initial_heading, len(sub_segment_lengths[i]), start))
                    self.distance = len(sub_segment_lengths[i]) - paths[-1][2]
                else:
                    paths.append(self.compute_segments(self.initial_heading, sub_segment_lengths[i], start))
                    self.distance = sub_segment_lengths[i] - paths[-1][2]
                start = paths[-1][0][1]
                self.theta_diff = self.get_theta_diff(paths[-1][0][0], paths[-1][0][1])
                self.initial_heading = self.get_initial_heading(paths[-1][0][0], paths[-1][0][1])
                
    
        return paths




    def close_enough(self, config1, config2, threshold=0.0):
        return np.linalg.norm(np.array(config1)-np.array(config2))<=threshold

  


    def compute_segments(self, initial_heading, distance, start):    
        if distance >= 2 * self.min_turning_radius and math.radians(initial_heading)>=0:
            end_point, length = self.path_segments(initial_heading, distance, ['L','R', 'L'], start)
            sequence = 'LRL'
        if distance >= 2 * self.min_turning_radius and math.radians(initial_heading)<0:
            end_point, length = self.path_segments(initial_heading, distance, ['R', 'L', 'R'], start)
            sequence = 'RLR'
        if distance >= self.min_turning_radius + abs(self.min_turning_radius * math.radians(self.theta_diff)) and math.radians(initial_heading)>=0:
            end_point, length = self.path_segments(initial_heading, distance, ['L', 'S', 'L'], start)
            sequence = 'LSL'
        if distance >= self.min_turning_radius + abs(self.min_turning_radius * math.radians(self.theta_diff)) and math.radians(initial_heading)<0: 
            end_point, length = self.path_segments(initial_heading, distance, ['R', 'S', 'R'], start)
            sequence = 'RSR'
        if distance >= self.min_turning_radius + abs(self.min_turning_radius * math.radians(self.theta_diff)) and (math.radians(initial_heading)+math.radians(self.theta_diff))>=0:
            end_point, length = self.path_segments(initial_heading, distance, ['L', 'S', 'R'], start)
            sequence = 'LSR'
        if distance >= self.min_turning_radius + abs(self.min_turning_radius * math.radians(self.theta_diff)) and (math.radians(initial_heading)+math.radians(self.theta_diff))<0: 
            end_point, length = self.path_segments(initial_heading, distance, ['R', 'S', 'L'], start)
            sequence = 'RSL'
        if abs(distance-self.sum_arcs)<=30.0:
            end_point, length = self.path_segments(initial_heading, distance, 'S', start)
            sequence = 'S' 
        return [((start), (end_point)), sequence, distance]
    
            
        #return min(paths, key=lambda x: sum(map(abs, [x[2]])))          



    def make_close_enough(self, start, end, sequence, length):
        t = 10
        while not self.close_enough(self.end_point, end, 0.0):
            final_end_point = self.interpolate_to_target_point(end, self.end_point)
            t-=1
            end = final_end_point
            if t==0:
                break
        if not self.close_enough(self.end_point, end, 0.0):
            if t==0:
                return [(start, end), sequence, length]
            return self.make_close_enough(start, end, sequence, length)
        else:
            return [(start, end), sequence, length]

    '''def iterate_nested_lists(self, nested_list):
        for item in nested_list:
            if isinstance(item, list):
                paths = self.iterate_nested_lists(item)
            else:
                return paths'''


    def is_last_list(self, nested_list, list_to_check):
        for item in nested_list:
            if isinstance(item, list):
                if self.is_last_list(item, list_to_check):
                    return True
            elif item == list_to_check:
                return nested_list.index(item) == len(nested_list) - 1
        return False

    


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
        interpolated_point = spline(0.5)
        
        return tuple(interpolated_point)




    def get_theta_diff(self, start, end):
        theta_diff = end[2] - start[2]
        return (theta_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize angle to [-pi, pi]




    def path_segments(self, initial_heading, distance, segment_type, start):
        # Compute all possible paths
        length = 0.0
        #theta_diff = self.get_theta_diff(start, end)
        '''while current_point != q1:
            path_segment = dubins_path(current_point, q1, turning_radius)               #work on this
            total_distance += path_segment[1]
            current_point = (current_point[0], current_point[1], current_point[2] + path_segment[0])'''

        for s in segment_type:
            if s=='L':
                #L
                center_x = start[0] + self.min_turning_radius * math.sin(math.radians(initial_heading))
                center_y = start[1] + self.min_turning_radius * math.cos(math.radians(initial_heading))

                arc_length = (math.radians(initial_heading) + (math.radians(self.theta_diff)+math.radians(self.turning_angle))) * self.min_turning_radius

                end_point_x = center_x + self.min_turning_radius * math.sin(math.radians(initial_heading) + (math.radians(self.theta_diff)+math.radians(self.turning_angle)))
                end_point_y = center_y + self.min_turning_radius * math.cos(math.radians(initial_heading) + (math.radians(self.theta_diff)+math.radians(self.turning_angle)))
                end_point_theta = math.radians(start[2]) + math.radians(initial_heading) + (math.radians(self.theta_diff)+math.radians(self.turning_angle))
                length += arc_length
            elif s=='S':    
                #S
                end_point_x = start[0] + distance * math.cos(math.radians(initial_heading))
                end_point_y = start[1] + distance * math.sin(math.radians(initial_heading))
                end_point_theta = math.radians(start[2]) + math.radians(initial_heading)
                length += distance
            elif s=='R':
                #R
                center_x = start[0] + self.min_turning_radius * math.sin(math.radians(initial_heading))
                center_y = start[1] + self.min_turning_radius * math.cos(math.radians(initial_heading))

                arc_length = (math.radians(initial_heading) + (math.radians(self.theta_diff)-math.radians(self.turning_angle))) * self.min_turning_radius

                end_point_x = center_x - self.min_turning_radius * math.sin(math.radians(initial_heading) + (math.radians(self.theta_diff)-math.radians(self.turning_angle)))
                end_point_y = center_y - self.min_turning_radius * math.cos(math.radians(initial_heading) + math.radians((self.theta_diff)-math.radians(self.turning_angle)))
                end_point_theta = math.radians(start[2]) - math.radians(self.initial_heading)  + (math.radians(self.theta_diff)-math.radians(self.turning_angle))
                length += arc_length

        return (end_point_x, end_point_y, end_point_theta), length
        #return min(paths, key=lambda x: sum(map(abs, [x[0]])))


    def process_seg_lens(self, d):
        segment_lengths, num_of_segs = self.get_segment_lengths(d)
        #print(segment_lengths)
        num = []
        
    
        lens = self.recur_lens(segment_lengths)
        #print(lens)
        num.append(len(lens))
        for i in lens:
            if isinstance(i, list):
                num.append(len(i))
                continue
            else:
                continue
        #print(num)
        return lens, num

def main():
    waypoints = []
    sequences = []
    with open('dubinsMPDS.csv', 'r') as file:
        # Create a CSV reader object
        csv_reader = csv.reader(file)
        paths = []
        # Iterate over each row in the CSV file
        for h, row in enumerate(csv_reader):
            if h==3:
                # Process each value in the row
                # Split the row based on tab ('\t')
                # Remove empty strings
                #split_row = [elem for elem in row if elem]
                split_row = row[0].split('\t')

                # Remove any leading or trailing whitespace from each element
                split_row = [elem.strip() for elem in split_row]
                # Convert elements to appropriate data types if needed
                # Assuming the elements from index 2 to the end are floats        
                if split_row[-1] == '':
                    split_row = split_row[:-1]
                data = [float(elem) for elem in split_row]
                
                for i, column in enumerate(data):
                    if i==3 and i+5<len(data):
                        start = ((data[i]), (data[i+1]), math.radians(data[i+2]))
                        end = ((data[i+3]), (data[i+4]), math.radians(data[i+5]))
                        
                        dubinspath = DubinsPath(start, end, float(data[1])) 
                        print(start, end, dubinspath.distance)
                        if dubinspath.distance>=50.0:
                            segment_lengths, num_of_segs = dubinspath.process_seg_lens(dubinspath.distance)
                            paths = dubinspath.segmentize_path(dubinspath.start, dubinspath.distance, num_of_segs, segment_lengths, paths, dubinspath.end_point)
                        else:
                            new_end, sequence, length = dubinspath.compute_segments(dubinspath.initial_heading, dubinspath.distance, dubinspath.start)
                            paths=[((dubinspath.start), (new_end)), sequence, length]
                        if h==5:
                            break
                if h==5:        
                    break
    print(paths)
    #print(waypoints, sequences, length)
if __name__ == '__main__':
    main()