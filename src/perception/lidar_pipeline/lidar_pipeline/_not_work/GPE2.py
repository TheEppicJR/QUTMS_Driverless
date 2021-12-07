# Modules
import time
import math
from typing import List, Tuple, NamedTuple
# Plotting Data
import matplotlib.pyplot as plt

# Line Fitting
from . import line_extraction

# Point Cloud Clustering
from . import DBSCAN

# Visualiser
from . import visualiser as vis

# Point object
from .point_obj import Point_Obj


# Returns the index to a bin that a point (x, y) maps to 
def get_bin(x: float, y: float):
    norm = math.sqrt((x**2)+(y**2))
    bin_index = math.floor(norm / BIN_SIZE)
    if norm % BIN_SIZE != 0:
        bin_index += 1
    if bin_index >= NUM_BINS:
        bin_index = -1
        #print("Point exceeds expected max range of LIDAR. bin_index:", bin_index)
    return math.floor(bin_index)


# Returns the index to a segment that a point maps to
def get_segment(x: float, y: float):
    return math.floor(math.atan2(y, x) / DELTA_ALPHA)



def points_to_seg_bin(point_cloud: List[Point_Obj]) -> List[List[List]]:
    # create segments[bins[points[]]]
    segments_bins: List[List[Point_Obj]] = [[[] for j in range(NUM_BINS)] for i in range(NUM_SEGMENTS)]
    for i in range(len(point_cloud)):
        point: Point_Obj = point_cloud[i]
        if point.bin != -1 and point.valid == True:
            segments_bins[point.segment][point.bin].append(
                [point.x, point.y, point.z]
            )
    return segments_bins


# Does not modify input array
def approximate_2D(segments_bins: List[List[List]]) -> List[List[List]]:
    # create segments[bins[points[]]]
    segments_approx: List[List[List]] = [[[] for j in range(NUM_BINS)] for i in range(NUM_SEGMENTS)]
    for i in range(NUM_SEGMENTS): # each segment
        for j in range(NUM_BINS): # each bin
            for k in range(len(segments_bins[i][j])): # each point
                point: Point_Obj = segments_bins[i][j][k] # get current point
                point_prime: list = [point.xy_distance, point.z]
                segments_approx[i][j].append(point_prime)
            segments_approx[i][j].sort(reverse=True)

            # if there are more than one prototype point in the segment - take first
            if len(segments_approx[i][j]) > 0: 
                segments_approx[i][j] = segments_approx[i][j][0]
    return segments_approx


# https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
def dist_points_3D(x_0: list, p_1: list, p_2: list) -> float:
    p_1_dist = [ (x_0[0] - p_1[0]), (x_0[1] - p_1[1]), (x_0[2] - p_1[2]) ]
    p_2_dist = [ (x_0[0] - p_2[0]), (x_0[1] - p_2[1]), (x_0[2] - p_2[2]) ]
    
    dist_cross = [
        p_1_dist[1]*p_2_dist[2] - p_2_dist[1]*p_1_dist[2], 
        -(p_1_dist[0]*p_2_dist[2] - p_2_dist[0]*p_1_dist[2]), 
        p_1_dist[0]*p_2_dist[1] - p_2_dist[0]*p_1_dist[1]
    ]
    dist_norm: float = math.sqrt(dist_cross[0]**2 + dist_cross[1]**2 + dist_cross[2]**2)

    p_norm: float = math.sqrt( 
        (p_2[0] - p_1[0])**2 + (p_2[1] - p_1[1])**2 + (p_2[2] - p_1[2])**2 
    )
    
    return dist_norm / p_norm


# Returns the start and end points (x, y, z) of a line
def line_to_end_points(line, segment_idx):
    start = line[2] # First point in line
    end = line[3] # Last point in line
    
    x_1 = start[0] * math.cos((segment_idx + 0.5) * DELTA_ALPHA)
    x_2 = end[0] * math.cos((segment_idx + 0.5) * DELTA_ALPHA)
    
    y_1 = start[0] * math.sin((segment_idx + 0.5) * DELTA_ALPHA)
    y_2 = end[0] * math.sin((segment_idx + 0.5) * DELTA_ALPHA)
    
    p_1 = [x_1, y_1, start[1]]
    p_2 = [x_2, y_2, end[1]]
    
    return [p_1, p_2]


# Modifies input
# Conservative approach implemented using T_D_MAX parameter
def label_points_5(segments_bins: List[List[List]], ground_lines: List[List[List]]) -> List[List[List]]:
    for i in range(NUM_SEGMENTS):
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
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k]
                is_ground = False
                line_height = ground_line[0] * (j * BIN_SIZE) + ground_line[1]
                if point[2] < line_height + 0.08: # Make this a constant
                    line = line_to_end_points(ground_line, seg_idx)
                    closest_dist = dist_points_3D(point, line[0], line[1])
                    for m in range(1, num_lines):
                        ground_line = ground_lines[seg_idx][m]
                        line = line_to_end_points(ground_line, seg_idx)
                        dist_to_line = dist_points_3D(point, line[0], line[1])
                        if (dist_to_line < closest_dist):
                            closest_dist = dist_to_line
                    dynamic_T_D_GROUND = 4*(j * BIN_SIZE)*math.tan(DELTA_ALPHA/2) # Solved for gradient of segment wrt points and distance
                    if (closest_dist < T_D_MAX and closest_dist < dynamic_T_D_GROUND):
                        is_ground = True
                segments_bins[i][j][k].append(is_ground)
    return segments_bins


# Modifies input
def label_points_6(segments_bins, ground_lines):
    for i in range(NUM_SEGMENTS):
        num_lines = len(ground_lines[i])
        seg_idx = i
        # If there is no ground line in current segment, find the closest one:
        if num_lines == 0:
            left_counter = i-1
            right_counter = i+1
            left_idx = (left_counter) % NUM_SEGMENTS
            right_idx = (right_counter) % NUM_SEGMENTS
            while left_idx != right_idx:
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
            if left_idx == right_idx: raise AssertionError("No ground lines found") # probably shouldnt have an error thrown 
        ground_line = ground_lines[seg_idx][0]
                
        for j in range(NUM_BINS):
            avg_point = [0, 0, 0]
            is_ground = False
            for k in range(len(segments_bins[i][j])):
                avg_point[0] += segments_bins[i][j][k][0]
                avg_point[1] += segments_bins[i][j][k][1]
                avg_point[2] += segments_bins[i][j][k][2]
                
                segments_bins[i][j][k].append(is_ground)

            line = line_to_end_points(ground_line, seg_idx)
            closest_dist = dist_points_3D(avg_point, line[0], line[1])
            
            for k in range(1, num_lines):
                line = ground_lines[seg_idx][k]
                dist_to_line = dist_points_3D(avg_point, line[0], line[1])
                if (dist_to_line < closest_dist):
                    closest_dist = dist_to_line
    return segments_bins


def non_ground_points(labelled_points):
    # Flatten parent array (remove bins)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    # Return all objects that are NOT flagged as ground
    return [point for point in labelled_points if point[3] == False]


# Ignoring height
def get_distance(point_a, point_b):
    # Distance
    return math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1]-point_a[1])**2)


def object_reconstruction_4(cluster_centers, segments_bins):
    cone_width = 0.15
    reconstructed_clusters = [[] for i in range(len(cluster_centers))]
    for i in range(len(cluster_centers)):
        cluster = cluster_centers[i]
        seg_idx = get_segment(cluster[0], cluster[1])
        bin_idx = get_bin(cluster[0], cluster[1])
        for j in range(seg_idx-1, (seg_idx+2) % NUM_SEGMENTS):
            for k in range(bin_idx-1, (bin_idx+2) % NUM_BINS):
                for m in range(len(segments_bins[j][k])):
                    point = segments_bins[j][k][m]
                    if get_distance(cluster, point) <= cone_width:
                        reconstructed_clusters[i].append(point)
                        break
                    
    return reconstructed_clusters
    

# I NEED TO COMPUTE THE CENTER OF A CLUSTER ONLY ONCE
# AND KEEP THIS VALUE. Instead of calculating it multiple times.
HORIZONTAL_RES = 0.2 * (math.pi / 180) # 0.2 degrees in between each point
VERTICAL_RES = 2 * (math.pi / 180) # 2 degrees in between each point

CONE_HEIGHT = 0.325 #m
CONE_WIDTH = 0.228 #m

def cone_filter(distance):
    if distance < 0.01: distance = 0.01
    return (1/2) * (CONE_HEIGHT / (2 * distance * math.tan(VERTICAL_RES / 2))) * (CONE_WIDTH / (2 * distance * math.tan(HORIZONTAL_RES / 2)))


FAR_X = 6 #m

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
            if abs(x_mean) < FAR_X: # only checks centre of scan for cones (noise filter)
                if (exp_point_count*(1 - ERROR_MARGIN) <= point_count <= exp_point_count*(1 + ERROR_MARGIN)):
                    cones.append([x_mean, y_mean])
                else:
                    cones.append([x_mean, y_mean]) # Remove when real-er data
    return cones


def get_ground_plane(point_cloud: List[Point_Obj]) -> List[list]:
    # might be able to modifiy segments directly if label points doesn't need it
    
    start_time: float = time.time()
    now: float = time.time()
    segments_bins: List[List[Point_Obj]] = points_to_seg_bin(point_cloud)
    print("points_to_seg_bin", time.time() - now)
    
    if VISUALISE: vis.plot_segments_bins(segments_bins, False)

    now = time.time()
    segments_bins_prototype: List[List[List]] = approximate_2D(segments_bins)
    print("approximate_2D", time.time() - now)

    now = time.time()
    ground_plane: List[list] = line_extraction.get_ground_plane(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    print("get_ground_plane", time.time() - now)

    if VISUALISE: vis.plot_ground_lines_3D(segments_bins_prototype, ground_plane, False)
    #if VISUALISE: vis.plot_segments_fitted(segments_bins_prototype, ground_plane)

    now = time.time()
    labelled_points = label_points_5(segments_bins, ground_plane)
    print("label_points", time.time() - now)

    if VISUALISE: vis.plot_labelled_points(labelled_points, ground_plane)

    now = time.time()
    object_points = non_ground_points(labelled_points)
    print("non_ground_points", time.time() - now)

    if VISUALISE: vis.plot_grid_2D(object_points)

    now = time.time()
    cluster_centers = DBSCAN.get_objects(object_points)
    print("get_objects", time.time() - now)

    now = time.time()
    reconstructed_clusters = object_reconstruction_4(cluster_centers, segments_bins)
    print("object_reconstruction", time.time() - now)

    if VISUALISE: vis.plot_reconstruction(reconstructed_clusters)

    now = time.time()
    cones = get_cones(reconstructed_clusters)
    print("get_cones", time.time() - now)

    if VISUALISE: vis.plot_cones(cones)

    # Could consider try except block to ensure plotting - even during failure
    if VISUALISE and DISPLAY: plt.show()
    
    print("Algorithm Time:", time.time() - start_time)
    return cones


def lidar_init(_visualise: bool, _display: bool, _figures_dir: str, _max_range: int=16):
    global LIDAR_RANGE
    global DELTA_ALPHA
    global NUM_SEGMENTS
    global BIN_SIZE
    global NUM_BINS
    global T_D_GROUND
    global T_D_MAX

    global VISUALISE
    global DISPLAY
    global FIGURES_DIR

    # Constants
    LIDAR_RANGE = _max_range # Max range of the LIDAR (m)
    DELTA_ALPHA = 2*math.pi / 128 # Angle of each segment # 2*pi / 64 implies 64 segments
    NUM_SEGMENTS = math.ceil(2*math.pi / DELTA_ALPHA) # Number of segments # 8
    BIN_SIZE = 0.25 # The length of a bin (in metres) # 1
    NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant

    T_D_GROUND = 0.1 # Maximum distance between point and line to be considered part of ground plane. # 2
    # T_D_PREV = 3           # Max distance of the first point of a line to the line previously fitted

    T_D_MAX = 100 # Maximum distance a point can be from origin to even be considered for ground plane labelling. Otherwise it's automatically labelled as non-ground.
    
    if (math.pi % DELTA_ALPHA != 0):
        raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle into an integer-number of segments.")
    
    VISUALISE = _visualise
    DISPLAY = _display
    if _display: VISUALISE = True
    FIGURES_DIR = _figures_dir


def lidar_main(point_cloud: List):
    # init_constants(_max_range) # move to be a one-time call in lidar init

    if VISUALISE:
        vis.init_constants(point_cloud, DELTA_ALPHA, LIDAR_RANGE, BIN_SIZE, VISUALISE, FIGURES_DIR)

    return get_ground_plane(point_cloud)

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
# 8. Implement logging
# 9. Consider removing completly empty segments at the start and see if that works!!!
#    I might still be treating this like I have a full circle of vision when really
#    It would only be 180 degrees
# 10. Investigate potential 'hacky point fix' in label_points function. I think this
#     has definitely been resolved though.
# 11. Review the non_ground_points function for the double flattening and see if 
#     this can be simpler. 
# 12. Object Reconstruction
# # You might want to sort points by x coords here
    # since that is how the clusters are sorted
    # This may reduce overall time
    # I'm gonna be hoping a point wont be in two clusters at the same time
            # therefore ill break after the first match for each point
            # Increases speed of algorithm