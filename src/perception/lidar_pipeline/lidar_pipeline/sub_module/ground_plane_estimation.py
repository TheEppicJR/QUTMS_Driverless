# Modules
import time
import math
import copy
import numpy as np

# Plotting Data
import matplotlib.pyplot as plt

# Line Fitting
from . import line_extraction

# Point Cloud Clustering
from . import DBSCAN

# Visualiser
from . import visualiser as vis

# Returns the index to a segment that a point maps to
def get_segment(x, y):
    return math.floor(math.atan2(y, x) / DELTA_ALPHA)

# Creates the set of segments for points
def init_segments():
    segments = [] 
    for i in range(NUM_SEGMENTS):
        segments.append([])
    return segments
    
# Assign each point to a segment 
def points_to_segment(points):
    segments = init_segments()
    for i in range(len(points)):
        x = points[i][0]
        y = points[i][1]
        s_index = get_segment(x, y)
        segments[s_index].append(points[i])
    return segments

def points_to_segment_2(points):
    segments = [[] for i in range(NUM_SEGMENTS)]
    for i in range(len(points)):
        point = points[i] # [x, y, z]
        seg_idx = get_segment(point[0], point[1])
        segments[seg_idx].append(point)
    return segments

# Creates a set of segments with bins
def init_bins():
    segments_bins = []
    for i in range(NUM_SEGMENTS):
        segments_bins.append([])
        for j in range(NUM_BINS):
            segments_bins[i].append([])
    #print("Number of bins:", len(segments_bins[0]))
    return segments_bins

# Returns true if the point (x, y) is in bin j
def in_bin(x, y, j):
    return (j*BIN_SIZE <= math.sqrt((x**2)+(y**2)) <= (j+1)*BIN_SIZE)

# Returns the index to a bin that a point (x, y) maps to 
def get_bin(x, y):
    norm = math.sqrt((x**2)+(y**2))
    bin_index = math.floor(norm / BIN_SIZE)
    if norm % BIN_SIZE != 0:
        bin_index += 1
    # If a point REALLY exceeds the limits, it shouldn't be included
    # Since processing points based on range will be done at the start,
    # this shouldn't be needed since the number of bins is calculated 
    # based on the specified range. Thus, by this point, all points should
    # belong a bin
    if bin_index >= NUM_BINS:
        bin_index = LIDAR_RANGE / BIN_SIZE - 1
        #print("Point exceeds expected max range of LIDAR. bin_index:", bin_index)
    return math.floor(bin_index)

# Assign each point to a bin
def points_to_bins(segments):
    segments_bins = init_bins()
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments[i])):
            x = segments[i][j][0]
            y = segments[i][j][1]
            bin_index = get_bin(x, y)
            segments_bins[i][bin_index].append(segments[i][j])
    return segments_bins

def points_to_bins_2(segments):
    segments_bins = [[[] for j in range(NUM_BINS)] for i in range(NUM_SEGMENTS)]
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments[i])):
            point = segments[i][j] # [x, y, z]
            bin_index = get_bin(point[0], point[1])
            segments_bins[i][bin_index].append(point)
    return segments_bins

# Creates a 2D approximation of the 3D points
def approximate_2D(segments_bins):
    segments_bins_2D = copy.deepcopy(segments_bins) 
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                x = segments_bins[i][j][k][0]
                y = segments_bins[i][j][k][1]
                z = segments_bins[i][j][k][2]
                point_prime = [math.sqrt(x**2 + y**2), z]
                segments_bins_2D[i][j][k] = point_prime
            # This should order the points by range for each bin:
            segments_bins_2D[i][j] = sorted(segments_bins_2D[i][j], key=lambda lam: lam[0])
            segments_bins_2D[i][j].reverse()
    return segments_bins_2D

def approximate_2D_2(segments_bins):
    segments_bins_2D = []
    for i in range(NUM_SEGMENTS):
        segments_bins_2D.append([])
        for j in range(NUM_BINS):
            segments_bins_2D[i].append([])
            for k in range(len(segments_bins[i][j])):
                x = segments_bins[i][j][k][0]
                y = segments_bins[i][j][k][1]
                z = segments_bins[i][j][k][2]
                point_prime = [math.sqrt(x**2 + y**2), z]
                segments_bins_2D[i][j].append(point_prime)
            segments_bins_2D[i][j].sort(reverse=True)
    return segments_bins_2D

# Modifies input array
def approximate_2D_3(segments_bins):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                x = segments_bins[i][j][k][0]
                y = segments_bins[i][j][k][1]
                z = segments_bins[i][j][k][2]
                point_prime = [math.sqrt(x**2 + y**2), z]
                segments_bins[i][j][k] = point_prime
            segments_bins[i][j].sort(reverse=True)
    return segments_bins

# Modifies input array
def approximate_2D_4(segments_bins):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k] # [x, y, z]
                point_prime = [math.sqrt(point[0]**2 + point[1]**2), point[2]]
                segments_bins[i][j][k] = point_prime
            segments_bins[i][j].sort(reverse=True)
    return segments_bins

# Modifies input array
def approximate_2D_5(segments_bins):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k] # [x, y, z]
                point_prime = [math.sqrt(point[0]**2 + point[1]**2), point[2]]
                segments_bins[i][j][k] = point_prime
            segments_bins[i][j].sort(reverse=True)
            # Prototype points
            if len(segments_bins[i][j]) > 0:
                segments_bins[i][j] = segments_bins[i][j][0]
    return segments_bins

# Reduces all points in a bin to a single prototype point
def prototype_points(segments_bins_2D):
    segments_bins_prototype = [] 
    for i in range(NUM_SEGMENTS):
        segments_bins_prototype.append([])
        for j in range(NUM_BINS):
            segments_bins_prototype[i].append([])
            if len(segments_bins_2D[i][j]) > 0:
                segments_bins_prototype[i][j] = segments_bins_2D[i][j][0]
    return segments_bins_prototype

# Modifies input
def prototype_points_2(segments_bins_2D):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            if len(segments_bins_2D[i][j]) > 0:
                segments_bins_2D[i][j] = segments_bins_2D[i][j][0]
    return segments_bins_2D

# Not sure if this is working correctly
# Should compare 3D distance between points and start of line point
def dist_points_3D_old(x1, x2, m, b):
    x2_z = m*x2[0] + b
    distance = math.sqrt((x2[0] - x1[0])**2 + (x2[1] - x1[1])**2 + (x2_z - x1[2])**2)
    return distance

# distance from point to line in 3D space
# def point_line_dist_3D():
#    pass

# might need to investigate line extraction distance from point to line, making large values.

# m
# b
# first point
# last point
# num points

# Can there be multiple lines PER segment?? I still don't know and this will
# directly affect how this function is written. 
# But assuming that there is multiple lines ...

# Conservative approach implemented using T_D_MAX parameter
def label_points_old(segments, ground_lines):
    labelled_points = copy.deepcopy(segments)

    # Assuming multiple lines in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        num_points = len(segments[i])
        for j in range(num_points): 
            point = segments[i][j]
            is_ground = False
            closest_line = None
            num_lines = len(ground_lines[i])
            if num_lines > 0:
                line = ground_lines[i][0]
                closest_line = line
                closest_dist = dist_points_3D_old(point, line[2], line[0], line[1])
                for k in range(1, num_lines):
                    line = ground_lines[i][k]
                    dist_to_line = dist_points_3D_old(point, line[2], line[0], line[1])
                    if (dist_to_line < closest_dist):
                        closest_line = line
                        closest_dist = dist_to_line
                hacky_point = [math.sqrt(point[0]**2 + point[1]**2), point[2]] # fix this
                point_to_line_dist = line_extraction.dist_point_line(hacky_point, closest_line[0], closest_line[1])
                if (closest_dist < T_D_MAX and point_to_line_dist < T_D_GROUND):
                    is_ground = True
            labelled_points[i][j].append(is_ground)
    return labelled_points

# https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
def dist_points_3D(x_0, x_1, x_2):
    numer = np.linalg.norm(np.cross(np.subtract(x_0, x_1), np.subtract(x_0, x_2)))
    denom = np.linalg.norm(np.subtract(x_2, x_1))
    distance = numer/denom
    return distance

def line_to_end_points(line, segment_idx):
    start = line[2] # First point in line
    end = line[3] # Last point in line
    r = np.linspace(start[0], end[0], 2)
    z = line[0] * r + line[1]
    x = r * math.cos((segment_idx + 0.5) * DELTA_ALPHA)
    y = r * math.sin((segment_idx + 0.5) * DELTA_ALPHA)
    x_1 = [x[0], y[0], z[0]]
    x_2 = [x[1], y[1], z[1]]
    return [x_1, x_2]

# Conservative approach implemented using T_D_MAX parameter
def label_points(segments, ground_lines):
    labelled_points = copy.deepcopy(segments)

    # Assuming multiple lines in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        num_points = len(segments[i])
        for j in range(num_points): 
            point = segments[i][j]
            is_ground = False
            num_lines = len(ground_lines[i])
            seg_idx = i
            # If there is ground line in corresponding segment
            # find the closest one
            if num_lines == 0:
                left_counter = i-1
                right_counter = i+1
                left_idx = (left_counter) % NUM_SEGMENTS
                right_idx = (right_counter) % NUM_SEGMENTS
                while left_idx != right_idx:
                    #print("i:", i, "left:", left_idx, "right:", right_idx)
                    if len(ground_lines[left_idx]) > 0:
                        seg_idx = left_idx
                        break
                    elif len(ground_lines[right_idx]) > 0:
                        seg_idx = right_idx
                        break
                    left_counter -= 1
                    right_counter += 1
                    left_idx = (left_counter) % NUM_SEGMENTS
                    right_idx = (right_counter) % NUM_SEGMENTS
                if left_idx == right_idx:
                    raise AssertionError("No ground lines found")
            line = line_to_end_points(ground_lines[seg_idx][0], seg_idx)
            closest_dist = dist_points_3D(point, line[0], line[1])
            for k in range(1, num_lines):
                line = ground_lines[seg_idx][k]
                dist_to_line = dist_points_3D(point, line[0], line[1])
                if (dist_to_line < closest_dist):
                    closest_dist = dist_to_line
            #point_to_line_dist = dist_points_3D(point, closest_line)
            if (closest_dist < T_D_MAX and closest_dist < T_D_GROUND):
                is_ground = True
            labelled_points[i][j].append(is_ground)
    return labelled_points

# Conservative approach implemented using T_D_MAX parameter
# Modifies input
def label_points_2(segments, ground_lines):
    # Assuming multiple lines can be in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments[i])): 
            point = segments[i][j]
            is_ground = False
            num_lines = len(ground_lines[i])
            seg_idx = i
            # If there is no ground line in current segment, find the closest one
            if num_lines == 0:
                left_counter = i-1
                right_counter = i+1
                left_idx = (left_counter) % NUM_SEGMENTS
                right_idx = (right_counter) % NUM_SEGMENTS
                while left_idx != right_idx:
                    #print("i:", i, "left:", left_idx, "right:", right_idx)
                    if len(ground_lines[left_idx]) > 0:
                        seg_idx = left_idx
                        break
                    elif len(ground_lines[right_idx]) > 0:
                        seg_idx = right_idx
                        break
                    left_counter -= 1
                    right_counter += 1
                    left_idx = (left_counter) % NUM_SEGMENTS
                    right_idx = (right_counter) % NUM_SEGMENTS
                if left_idx == right_idx:
                    raise AssertionError("No ground lines found")
            line = line_to_end_points(ground_lines[seg_idx][0], seg_idx)
            closest_dist = dist_points_3D(point, line[0], line[1])
            for k in range(1, num_lines):
                line = ground_lines[seg_idx][k]
                dist_to_line = dist_points_3D(point, line[0], line[1])
                if (dist_to_line < closest_dist):
                    closest_dist = dist_to_line
            dynamic_T_D_GROUND = (BIN_SIZE*(get_bin(point[0], point[1])))*math.tan(DELTA_ALPHA/2) # Solved for gradient of segment wrt points and distance
            if (closest_dist < T_D_MAX and closest_dist < dynamic_T_D_GROUND):
                is_ground = True
            segments[i][j].append(is_ground)
    return segments

# Conservative approach implemented using T_D_MAX parameter
# Modifies input
def label_points_3(segments, ground_lines):
    # Assuming multiple lines can be in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments[i])): 
            point = segments[i][j]
            is_ground = False
            num_lines = len(ground_lines[i])
            seg_idx = i
            # If there is no ground line in current segment, find the closest one
            if num_lines == 0:
                left_counter = i-1
                right_counter = i+1
                left_idx = (left_counter) % NUM_SEGMENTS
                right_idx = (right_counter) % NUM_SEGMENTS
                while left_idx != right_idx:
                    #print("i:", i, "left:", left_idx, "right:", right_idx)
                    if len(ground_lines[left_idx]) > 0:
                        seg_idx = left_idx
                        break
                    elif len(ground_lines[right_idx]) > 0:
                        seg_idx = right_idx
                        break
                    left_counter -= 1
                    right_counter += 1
                    left_idx = (left_counter) % NUM_SEGMENTS
                    right_idx = (right_counter) % NUM_SEGMENTS
                if left_idx == right_idx:
                    raise AssertionError("No ground lines found")
            ground_line = ground_lines[seg_idx][0]
            bin_index = get_bin(point[0], point[1])
            line_height = ground_line[0] * bin_index + ground_line[1]
            if point[2] < line_height + 0.08: # Make this a constant
                line = line_to_end_points(ground_line, seg_idx)
                closest_dist = dist_points_3D(point, line[0], line[1])
                for k in range(1, num_lines):
                    line = ground_lines[seg_idx][k]
                    dist_to_line = dist_points_3D(point, line[0], line[1])
                    if (dist_to_line < closest_dist):
                        closest_dist = dist_to_line
                # bin_index + 2 is extra leeway
                dynamic_T_D_GROUND = 4*(BIN_SIZE*(bin_index))*math.tan(DELTA_ALPHA/2) # Solved for gradient of segment wrt points and distance
                if (closest_dist < T_D_MAX and closest_dist < dynamic_T_D_GROUND):
                    is_ground = True
            segments[i][j].append(is_ground)
    return segments

# Conservative approach implemented using T_D_MAX parameter
# Modifies input
def label_points_2_old(segments, ground_lines):
    # Assuming multiple lines in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        # For every point in each segment. Perhaps make an array of constants of point counts?
        for j in range(len(segments[i])):
            point = segments[i][j]
            is_ground = False
            closest_line = None
            num_lines = len(ground_lines[i])
            if num_lines > 0:
                line = ground_lines[i][0]
                closest_line = line
                closest_dist = dist_points_3D(point, line[2], line[0], line[1])
                for k in range(1, num_lines):
                    line = ground_lines[i][k]
                    dist_to_line = dist_points_3D(point, line[2], line[0], line[1])
                    if (dist_to_line < closest_dist):
                        closest_line = line
                        closest_dist = dist_to_line
                hacky_point = [math.sqrt(point[0]**2 + point[1]**2), point[2]] # fix this
                point_to_line_dist = line_extraction.dist_point_line(hacky_point, closest_line[0], closest_line[1])
                if (closest_dist < T_D_MAX and point_to_line_dist < T_D_GROUND):
                    is_ground = True
            segments[i][j].append(is_ground)
    return segments

def non_ground_points(labelled_points):
    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    object_points = []
    for i in range(len(labelled_points)):
        point = labelled_points[i]
        if point[3] == False:
            object_points.append([point[0], point[1], point[2]])
    return object_points

def non_ground_points_2(labelled_points):
    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    # Return all objects that are NOT flagged as ground
    return [point for point in labelled_points if point[3] == False]

# Ignoring height
def get_distance(point_a, point_b):
    # Distance
    return math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1]-point_a[1])**2)

def object_reconstruction(cluster_centers, points):
    cone_width = 0.15
    reconstructed_clusters = []
    # You might want to sort points by x coords here
    # since that is how the clusters are sorted
    # This may reduce overall time
    for i in range (len(cluster_centers)):
        reconstructed_clusters.append([])
    for i in range(len(points)):
        point = points[i]
        for j in range(len(cluster_centers)):
            cluster_center = cluster_centers[j]
            # I'm gonna be hoping a point wont be in two clusters at the same time
            # therefore ill break after the first match for each point
            # Increases speed of algorithm
            if get_distance(cluster_center, point) <= cone_width:
                reconstructed_clusters[j].append(point)
                break;
    return reconstructed_clusters

def object_reconstruction_2(cluster_centers, points):
    cone_width = 0.15
    reconstructed_clusters = [[] for i in range(len(cluster_centers))]
    # You might want to sort points by x coords here
    # since that is how the clusters are sorted
    # This may reduce overall time
    for i in range(len(points)):
        point = points[i]
        for j in range(len(cluster_centers)):
            # I'm gonna be hoping a point wont be in two clusters at the same time
            # therefore ill break after the first match for each point
            # Increases speed of algorithm
            if get_distance(cluster_centers[j], point) <= cone_width:
                reconstructed_clusters[j].append(point)
                break;
    return reconstructed_clusters

# I NEED TO COMPUTE THE CENTER OF A CLUSTER ONLY ONCE
# AND KEEP THIS VALUE. Instead of calculating it multiple times.
def cone_filter(distance):
    cone_height = 0.325
    cone_width = 0.228
    horizontal_res = 0.2 * (math.pi / 180) # 0.2 degrees in between each point
    vertical_res = 2 * (math.pi / 180) # 2 degrees in between each point
    exp_point_count = (1/2) * (cone_height / (2 * distance * math.tan(vertical_res / 2))) * (cone_width / (2 * distance * math.tan(horizontal_res / 2)))
    return exp_point_count

def get_cones(reconstructed_clusters):
    cones = []
    ERROR_MARGIN = 0.2 # Constant
    for i in range(len(reconstructed_clusters)):
        point_count = len(reconstructed_clusters[i])
        if point_count >= 1:
            x_cluster = [coords[0] for coords in reconstructed_clusters[i]]
            y_cluster = [coords[1] for coords in reconstructed_clusters[i]]
            # Univserity of melbourne used z as well
            #z_cluster = [coords[1] for coords in reconstructed_clusters[i]]
            x_mean  = sum(x_cluster) / len(x_cluster)
            y_mean  = sum(y_cluster) / len(y_cluster)
            #z_mean  = sum(y_cluster) / len(y_cluster)
            distance = math.sqrt(x_mean ** 2 + y_mean ** 2)
            # Rule based filter
            exp_point_count = cone_filter(distance)
            #print(exp_point_count, point_count)
            if (exp_point_count*(1 - ERROR_MARGIN) <= point_count <= exp_point_count*(1 + ERROR_MARGIN)):
                cones.append([x_mean, y_mean])
            else:
                cones.append([x_mean, y_mean]) # Remove when real-er data
    return cones

def benchmarking(points):
    print("here")
    # Optimised
    start_time = time.time()
    segments = points_to_segment_2(points)
    time_1 = time.time() - start_time

    # Optimised 
    start_time = time.time()
    segments_bins = points_to_bins_2(segments)
    time_2 = time.time() - start_time

    # Optimised
    #start_time = time.time()
    #segments_bins_2D = approximate_2D_4(segments_bins)
    #time_3 = time.time() - start_time

    # Optimised
    #start_time = time.time()
    #segments_bins_prototype = prototype_points_2(segments_bins_2D)
    #time_4 = time.time() - start_time

    # - Merging 2 Components -

    # Optimised: Modifies input array
    start_time = time.time()
    segments_bins_prototype = approximate_2D_5(segments_bins)
    time_3 = time.time() - start_time
    time_4 = 0 # Component Merged.

    start_time = time.time()
    ground_lines = line_extraction.get_ground_plane(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    time_5 = time.time() - start_time

    # Improved: Modifies input 
    # You need to re-optimise the new label points function
    start_time = time.time()
    labelled_points = label_points_2(segments, ground_lines)
    time_6 = time.time() - start_time

    # Optimised
    start_time = time.time()
    object_points = non_ground_points_2(labelled_points)
    time_7 = time.time() - start_time

    # Optimised? (should be quicker for larger array)
    start_time = time.time()
    #clusters, noise, cluster_centers = DBSCAN.init_DBSCAN(object_points)
    cluster_centers = DBSCAN.init_DBSCAN_2(object_points)
    time_8 = time.time() - start_time

    # Optimised (A tiny bit faster)
    start_time = time.time()
    reconstructed_clusters = object_reconstruction_2(cluster_centers, points)
    time_9 = time.time() - start_time

    # Already Optimised
    start_time = time.time()
    cones = get_cones(reconstructed_clusters)
    time_10 = time.time() - start_time

    return [time_1, time_2, time_3, time_4, time_5, time_6, time_7, time_8, time_9, time_10]

def get_ground_plane_old(points):
    # This is still using the unoptimised functions (for visualisation purposes)
    segments = points_to_segment(points)
    segments_bins = points_to_bins(segments)
    segments_bins_2D = approximate_2D_4(segments_bins)
    segments_bins_prototype = prototype_points(segments_bins_2D)
    ground_lines = line_extraction.extract_lines(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    # label_points labels ALL points, but don't we only need to append non-ground points (would speed up algorithm)
    labelled_points = label_points_2(segments, ground_lines)
    object_points = non_ground_points(labelled_points)
    clusters, noise, cluster_centers = DBSCAN.init_DBSCAN(object_points)
    reconstructed_clusters = object_reconstruction(cluster_centers, points)
    cones = get_cones(reconstructed_clusters)

    # Plotting
    # Could have plot after each thing above ^ and then have all of that
    # inside of a try() except with plt.show() guaranteed to occur.
    if VISUALISE : visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, ground_lines, labelled_points, object_points, clusters, noise, reconstructed_clusters, cones)

    #print("\n\n\n", object_points)
    print(cones)
    return cones

def get_ground_plane(point_cloud):
    # might be able to modifiy segments directly if label points doesn't need it
    segments = points_to_segment_2(point_cloud)
    if VISUALISE: vis.plot_segments(segments)

    segments_bins = points_to_bins_2(segments)
    if VISUALISE: vis.plot_segments_bins(segments_bins, False)

    segments_bins_prototype = approximate_2D_5(segments_bins)

    ground_plane = line_extraction.get_ground_plane(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    if VISUALISE: vis.plot_ground_lines_3D(segments_bins_prototype, ground_plane, False)
    #if VISUALISE: vis.plot_segments_fitted(segments_bins_prototype, ground_plane)

    labelled_points = label_points_3(segments, ground_plane)
    if VISUALISE: vis.plot_labelled_points(labelled_points, ground_plane)

    object_points = non_ground_points_2(labelled_points)
    if VISUALISE: vis.plot_grid_2D(object_points)

    cluster_centers = DBSCAN.get_objects(object_points)

    reconstructed_clusters = object_reconstruction_2(cluster_centers, point_cloud)
    if VISUALISE: vis.plot_reconstruction(reconstructed_clusters)

    cones = get_cones(reconstructed_clusters)
    if VISUALISE: vis.plot_cones(cones)

    # Could consider try except block to ensure plotting - even during failure
    if VISUALISE and DISPLAY: plt.show() 
    return cones


def visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, ground_lines, labelled_points, object_points, clusters, noise, reconstructed_clusters, cones):
    # Could add a try except here so I can always see the plots even if error occurs
    #vis.plot_data_2D()
    vis.plot_data_3D()
        #vis.plot_segments(segments)
        #vis.plot_segments_bins(segments_bins, False) # Laggy due to bin count (Set False to decrease lag)
    #vis.plot_segments_bins_2D(segments_bins_2D, False) # Laggy due to bin count (Set False to decrease lag)
    #vis.plot_segments_bins_2D_3D(segments_bins_2D, False) # Laggy due to bin count (Set False to decrease lag)
    #vis.plot_segments_bins_prototype_3D(segments_bins_prototype, False) # Laggy due to bin count (Set False to decrease lag)
        #vis.plot_ground_lines_3D(segments_bins_prototype, ground_lines, False) # Laggy due to bin count (Set False to decrease lag)
    #vis.plot_segments_fitted(segments_bins_prototype, ground_lines) # Creates many figures (One for each segment)
    vis.plot_labelled_points(labelled_points, ground_lines)
    vis.plot_grid_2D(object_points)
    vis.plot_clusters(clusters, noise)
    vis.plot_reconstruction(reconstructed_clusters)
    vis.plot_cones(cones)
    plt.show()

# new_x.append(norm[k] * math.cos((i + 0.5) * DELTA_ALPHA))
# new_y.append(norm[k] * math.sin((i + 0.5) * DELTA_ALPHA))

def init_constants():
    global LIDAR_RANGE
    global DELTA_ALPHA
    global NUM_SEGMENTS
    global BIN_SIZE
    global NUM_BINS
    global T_D_GROUND
    global T_D_MAX

    # Constants
    LIDAR_RANGE = 15 # Max range of the LIDAR # 100 # in metres
    DELTA_ALPHA = 2*math.pi / 128 # Angle of each segment # 45 deg
    NUM_SEGMENTS = math.ceil(2*math.pi / DELTA_ALPHA) # Number of segments # 8
    BIN_SIZE = 0.25 # The length of a bin (in metres) # 1
    NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant

    T_D_GROUND = 0.1 # Maximum distance between point and line to be considered part of ground plane. # 2
    # T_D_PREV = 3           # Max distance of the first point of a line to the line previously fitted

    T_D_MAX = 100 # Maximum distance a point can be from origin to even be considered for ground plane labelling. Otherwise it's automatically labelled as non-ground.
    
    if (math.pi % DELTA_ALPHA != 0):
        raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle into an integer-number of segments.")

def lidar_main(point_cloud, _visualise, _display, benchmark, _figures_dir):
    init_constants()
    if benchmark:
        return benchmarking(point_cloud)
    else:
        global VISUALISE
        global DISPLAY
        VISUALISE = _visualise
        DISPLAY = _display
        if _display: VISUALISE = True

        # Remove this when a max range for the Lidar has been decided on
        global LIDAR_RANGE

        for i in range(len(point_cloud) - 1, -1, -1):
            point = point_cloud[i]
            if math.sqrt(point[0]**2 + point[1]**2 + point[2]**2) >= LIDAR_RANGE:
                point_cloud.remove(point)
        print(len(point_cloud))
        # values = []
        # for i in range(len(point_cloud)):
        #    values.append(math.sqrt(point_cloud[i][0] ** 2 + point_cloud[i][1] ** 2))
        # LIDAR_RANGE = math.ceil(max(values))

        print("Max xy norm:", LIDAR_RANGE) # Max value of the norm of x and y (excluding z)

        if VISUALISE:
            FIGURES_DIR = _figures_dir
            vis.init_constants(point_cloud, DELTA_ALPHA, LIDAR_RANGE, BIN_SIZE, VISUALISE, FIGURES_DIR)

        cones = get_ground_plane(point_cloud)
        return cones

#lidar_main(test_data, True, True, False)

# Notes
#   - Explore Python threading for this entire process
#   - Constant M: M must be an int, the value error below it
#     checks for this (I think).
#   - get_segment(): Returns negatives but Python lists handle 
#     this and wraps around.
#   - Var segments: segments[0] is the set of all points in the 
#     first segment.
#   - get_segment(): I assume, but should check that this always 
#     returns an int
#   - Constant BIN_SIZE: Might not be a value in metres, depends 
#     on the encoding of LIDAR data. 
#   - in_bin(): Note: bins don't need to be defined by the segment 
#     they're in. They're defined by the distance from the origin.
#     Each segment will have the same number of bins spaced at the 
#     same distances away from origin. 
#     This function is currently not needed. 
#   - Constant NUM_BINS: # This NEEDS to  be an int. Perhaps I could 
#     round down or have the last bin be smaller?
#   - approximate_2D(): This 3D to 2D conversion is a propety of 
#     the bins. The greater number of bins, the more accurate the 
#     approximation. This conversion is done by getting the distance 
#     x and y are from the origin and a prime point is simply a 2 
#     dimensional vector of this norm and the z height. This function 
#     also orders the points in each bin in ascending range 
#     (distance from origin)
#   - prototype_points(): point-to-bin mapping is not one-to-one, 
#     and P_(b_j^s)^' may contain more than one point, or no points 
#     at all. A prototype point will be the point within a bin with
#     the lowest z value (height) as this point is most likely to lie
#     on the plane.
#   - T_D_GROUND: Perhaps this should be dynamic and scale with the 
#     size of bins / size of segments, rather than being static. Same
#     goes for T_D_MAX. Maybe it should be a lil dynamic too
#   - Points that constructed the ground plane lines should already be
#     labelled as ground, rather than being relablled. 
#   - num_points = len(points) # make this contanst above if its used #     a lot throughout code
#   - bruh I was about to write this exact comment ^ create a constant for 
#     the number of points and reference it instead. 
#   - I should consider sorting the points early on, I think multiple
#     parts of the program would benefit by this / already need to do this
#   - Program encounters issues if point is at EXACT xy origin, because what
#     segment is this even in? This is practically impossible to encounter however.
#   - Update len() - 1 notation to [-1] where applicable.

# 7. T_D_PREV has been updated to a dynamic mathematical relation that increases
#    as points get further from the origin. HOWEVER, T_D_PREV is used in
#    ground_plane_estimation, not here anymore. 