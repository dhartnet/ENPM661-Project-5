# ENPM661 Project 5 Submission
# Rey Roque-Pérez and Dustin Hartnett

# Github Repository: https://github.com/dhartnet/ENPM661-Project-5

import cv2
import time
import random
import numpy as np
import heapq
from scipy.spatial import cKDTree

# Create the obstacle map
def create_map():
    i = 0
    j = 0

    # obs 1
    x11 = 150 // res - clearance - radius
    x12 = 175 // res + clearance + radius
    y11 = 0 // res - clearance - radius
    y12 = 100 // res + clearance + radius

    # obs 2

    x31 = 370 // res - clearance - radius
    x32 = 470 // res + clearance + radius
    y31 = 30 // res - clearance - radius
    y32 = 130 // res + clearance + radius

    case = False

    while case == False:

        # obs 1
        if i >= x11 and i <= x12 and j >= y11 and j <= y12:
            map_array[i,j] = 1

        # obs 2
        elif i >= x31 and i <= x32 and j >= y31 and j <= y32:
            map_array[i,j] = 1

        i = i + 1

        if i == 600 // res and j == 199 // res:

            case = True

        if i == 600 // res:
            i = 0
            j = j + 1

###---------------------------------------###
# Check to see if node in question lies within obstacle space
# Return 'False' if in free space, 'True' if in an obstacle or outside the boundaries
def inObstacle(maybeNode):
    node = tuple(maybeNode)
    i = int(node[0])
    j = int(node[1])

    # check if in map
    if i < clearance + radius or i > spaceX - clearance - radius or j < clearance + radius or j > spaceY - clearance - radius:
        return True
    
    if map_array[i,j] == 1:
        return True

    return False

# Non-holonomic move function
def newNodes(nodeState, clearance, radius, u1, u2): # (node, lower wheel velocity, higher wheel velocity) speeds in RPM
  # Extract node information from nodeState value
  node = tuple(nodeState) # Rounded node
  xi = int(node[0]) # Rounded node
  yi = int(node[1]) # Rounded node
  thetai = node[2]*30 # deg # Rounded node

  u1 = np.pi * u1 / 30 # (rad/s)
  u2 = np.pi * u2 / 30# (rad/s)

  # Define action set from wheel rotational velocities
  actions = [[0, u1], [u1, 0], [u1, u1], [0, u2], [u2, 0], [u2, u2], [u1, u2], [u2, u1]]

  # make empty lists to fill with values
  newNodes = [] # new node information
  c2c = [] # list of costs to come for each new node from parent node

  # define constants and counter values
  t = 0 # time (sec)
  dt = 0.1 # time step for integrating
  r =  3.3 # wheel radius cm
  L =  28.7 # robot diameter cm

  for action in actions:
    ul = action[0] # left wheel rotation velocity (rad/s)
    ur = action[1] # right wheel rotation velocity (rad/s)
    theta = np.pi * thetai / 180 # orientation in radians
    x = xi # set x value to initial node value before integration
    y = yi # set y value to initial node value before integration

    # we let each action run for one second, with a time step of 0.1 seconds for integration calculation
    while t <= 1.0:
      t = t + dt
      x = x + (r/2) * (ul + ur) * np.cos(theta) * dt
      y = y + (r/2) * (ul + ur) * np.sin(theta) * dt
      theta = theta + (r/L) * (ur - ul) * dt

    t = 0
    c2c = np.sqrt((xi - x)**2 + (yi - y)**2) # cost to come is linear displacement, not calculating distance 
    newX = int(round(x,0))
    newY = int(round(y,0))

    new_theta = 180 * theta / np.pi #deg
    if new_theta < 0:
      new_theta = 360 + new_theta

    if new_theta >= 360:
      new_theta = new_theta - 360

    theta = new_theta  
    new_theta = int((round(new_theta, 0) % 360)/30) # Rounded theta for comparing nodes

    # v = round((r/2) * (ul + ur), 2)
    # ang = (r/L) * (ur - ul)
    # ang = round(ang, 2) # rpm

    v = (r/2) * (ul + ur)
    ang = (r/L) * (ur - ul)

    if not (newX < clearance + radius or newX > spaceX - clearance - radius or newY < clearance + radius or newY > spaceY - clearance - radius):
      newNodes.append(((newX, newY, new_theta), round(c2c), (v, ang))) # outputs node, cost to come, and associated linear and ang velocity to get to each node (to be sent to robot)

  return newNodes

# Samples random points in the map
def sample_points(num_samples, space_limits):
    min_x, max_x, min_y, max_y = space_limits
    sampled_points = [start_point, goal_point]
    for _ in range(num_samples-2):
        x = random.randint(min_x, max_x)
        y = random.randint(min_y, max_y)
        for theta_index in range(12):  # Iterate over 12 theta values
            sampled_points.append((round(x, 0), round(y, 0), theta_index))
    return sampled_points

# Samples a uniform grid of points in the map
def sample_points_uniform(num_samples, space_limits, start_node, end_node):

    min_x, max_x, min_y, max_y = space_limits

    sampled_points = [(start_node,0,(0,0)), (end_node,0,(0,0))]
    
    # Calculate the aspect ratio of the space
    aspect_ratio = (max_x - min_x) / (max_y - min_y)
    
    num_samples = num_samples - 2

    # Calculate the number of samples along each axis
    num_samples_x = int((num_samples * aspect_ratio) ** 0.5)
    num_samples_y = int(num_samples / num_samples_x)
    
    # Calculate the step size for sampling along x and y axes
    step_x = (max_x - min_x) / (num_samples_x - 1)
    step_y = (max_y - min_y) / (num_samples_y - 1)
    
    # Generate uniformly sampled points
    for i in range(num_samples_x):
        for j in range(num_samples_y):
            x = min_x + i * step_x
            y = min_y + j * step_y
            for theta_index in range(12):  # Iterate over 12 theta values
                sampled_points.append(((round(x, 0), round(y, 0), theta_index),0,(0,0)))
    
    # Return the sampled points
    return sampled_points

# Calculates distance between two points
def calculate_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

# Takes the sampled nodes list and generates a connected graph
def generate_adjacency_list(sampled_points, max_neighbors=4, max_distance=30, threshold=4):
    adjacency_list = {}
    # Extracting only the relevant tuple elements from sampled_points
    only_points = [point[0] for point in sampled_points]
    point_array = np.array(only_points)  # Convert to NumPy array
    
    tree = cKDTree(point_array)
    
    for point in sampled_points:
        indices = tree.query_ball_point(point[0], max_distance)
        possible_neighbors = [tuple(neighbor) for neighbor in point_array[indices] if not np.array_equal(neighbor, point[0])]
        random.shuffle(possible_neighbors)  # Shuffle the list of neighbors before rounding
        valid_neighbors = connect_neighbors(point[0], possible_neighbors, threshold, max_neighbors)
        if len(valid_neighbors) > max_neighbors:
            valid_neighbors = random.sample(valid_neighbors, max_neighbors)  # Randomly select max_neighbors from valid_neighbors
        adjacency_list[point[0]] = valid_neighbors
    
    return adjacency_list

# Finds the possible neighbors for a given point
def connect_neighbors(point, possible_neighbors, threshold, max_neighbors):
    neighbors = []
    moves = newNodes(point, clearance, radius, rpm1, rpm2)
#    moves = [move[0] for move in moves]
    count = 0  # Variable to keep track of the number of neighbors found
    for neighbor in possible_neighbors:    
        if count >= max_neighbors:
            break  # Exit the loop if maximum neighbors reached
        for move in moves:
            distance = calculate_distance(neighbor, move[0])
            if distance <= threshold:
                neighbors.append((neighbor,move[1],move[2]))
                count += 1
                if count >= max_neighbors:
                    break  # Exit the inner loop if maximum neighbors reached
                break  # Exit the inner loop if a valid neighbor is found
    
    return neighbors

# Tries to delete unneeded nodes
def smooth_path(path, max_distance=60, threshold=4):
    i = 0
    while i < len(path) - 1:
        point = path[i]

        # Find all subsequent points within max_distance from the current point
        nearby_points = [next_point for next_point in path[i+1:] if calculate_distance(point, next_point) <= max_distance]

        # If no nearby points are found, move to the next point in the path
        if not nearby_points:
            i += 1
            continue

        # Find the furthest move among the nearby points
        furthest_move = None
        max_move_distance = -1
        moves = newNodes(point, clearance, radius, rpm1, rpm2)
        for move in moves:
            if not inObstacle(move[0]) and move[0] in nearby_points:
                move_distance = calculate_distance(point, move[0])
                if move_distance > max_move_distance:
                    furthest_move = move[0]
                    max_move_distance = move_distance

        # If a furthest move is found, update the path
        if furthest_move and furthest_move != path[i+1]:
            # Remove all points after the current point and prior to the next point
            path = path[:i+1] + path[path.index(furthest_move):]
        else:
            i += 1  # Move to the next point if no furthest move found

    return path

# Creates a list of the obstacles in the workspace
def obstacle_space():
    obstacle_list = []

    # obs 1
    x11 = 150 // res* conversion
    x12 = 175 // res* conversion
    y11 = 0 // res* conversion
    y12 = 100 // res* conversion

    # obs 3

    x31 = 370 // res* conversion
    x32 = 470 // res* conversion
    y31 = 30 // res* conversion
    y32 = 130 // res* conversion

    obstacle_list.append(((0,0),(visX,visY)))
    obstacle_list.append(((x11,y11),(x12,y12)))
    obstacle_list.append(((x31,y31),(x32,y32)))

    return obstacle_list

# Populates the canvas with the obstacles
def draw_obstacles(canvas, obstacles, video_output):
    for obstacle in obstacles:      
        if len(obstacle) == 2 and obstacle != ((0,0),(visX,visY)):
            start_x = obstacle[0][0]
            # Invert y-value
            start_y = obstacle[0][1]
            end_x = obstacle[1][0]
            # Invert y-value
            end_y = obstacle[1][1]
            start_coordinate = (start_x, start_y)
            end_coordinate = (end_x, end_y)
            cv2.rectangle(canvas, pt1=start_coordinate, pt2=end_coordinate, color=(0, 0, 0), thickness=-1)
    canvas1 = cv2.resize(canvas, (resizeX, resizeY))            
    #cv2.imshow('A*', canvas1)
    video_output.write(canvas1)
    #cv2.waitKey(0)
    return

# Draws the connections between nodes
def draw_connections(image, adjacency_list, color=(0, 255, 0), thickness=4):
    for point, neighbors in adjacency_list.items():
        for neighbor in neighbors:
            cv2.line(image, (int(point[0]*conversion), int(point[1]*conversion)), (int(neighbor[0][0]*conversion), int(neighbor[0][1]*conversion)), color, thickness)
    image1 = cv2.resize(image, (resizeX, resizeY)) 
    #cv2.imshow('A*', image1)
    video_output.write(image1)
    add_blank_frames(image1,video_output,60,3)
    #cv2.waitKey(0) 
    return

# Draws the sampled points
def draw_points(image, points, color=(255, 0, 0), radius=20):
    for point in points:
        x = int(point[0][0] * conversion)
        y = int(point[0][1] * conversion)
        cv2.circle(image, (x, y), radius, color, -1)
    image1 = cv2.resize(image, (resizeX, resizeY))
    #cv2.imshow('A*', image1)
    video_output.write(image1)
    add_blank_frames(image1,video_output,60,1)
    #cv2.waitKey(0)
    return

# Draws start node and end node
def draw_nodes(canvas, start_node, goal_node):
    cv2.rectangle(canvas, (conversion * start_node[0] - 30, conversion * start_node[1] - 30), (conversion * start_node[0] + 30, conversion * start_node[1] + 30), color=(0, 250, 0), thickness=cv2.FILLED)
    cv2.rectangle(canvas, (conversion * goal_node[0] - 30, conversion * goal_node[1] - 30), (conversion * goal_node[0] + 30, conversion * goal_node[1] + 30), color=(0, 0, 255), thickness=cv2.FILLED)

# Adds blank frames for x amount of seconds at end of video
def add_blank_frames(canvas, video_output, fps, seconds):
    blank_frames = int(fps * seconds)
    canvas1 = cv2.resize(canvas, (resizeX, resizeY))            
    for _ in range(blank_frames):
        video_output.write(canvas1)
    return

# Heuristic
def euclidean_distance(node1, node2):
    x1, y1, _ = node1
    x2, y2, _ = node2
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Reconstruct the path from the came_from dictionary
def reconstruct_path(came_from, current_node):
    total_path = [current_node]
    while current_node in came_from:
        current_node = came_from[current_node]
        total_path.append(current_node)
    return total_path[::-1]

# Runs A* on the given graph. Doesn't check for collision or explore the obstacle space. 
# Just navigates the given predefined graph. Not the focus of this project.
def astar(start, goal, graph):
    open_set = []
    closed_set = set()
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = euclidean_distance(start, goal)
    heapq.heappush(open_set, (f_score[start], start))

    path = []  # List to store the path
    v_ang_list = []  # List to store (v, ang)

    while open_set:
        current_f, current_node = heapq.heappop(open_set)

        if current_node == goal:
            path = reconstruct_path(came_from, current_node)
            v_ang_list = [graph[node][-1][-1] for node in path]  # Extract (v, ang) from the path
            return path, v_ang_list

        closed_set.add(current_node)

        for neighbor, cost, v_ang in graph[current_node]:
            tentative_g_score = g_score[current_node] + cost

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + euclidean_distance(neighbor, goal)
                if neighbor not in closed_set:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, None  # No path found

# Lazy PRM function
# Runs A* on sampled nodes without checking for collisions
# Checks generated path for collision and colliding nodes from graph
# Repeats this process until a valid path is found or the graph becomes disconnected
def run_lazy_prm(adjacency_list):
    ti = time.time()
    while True:
        path, v_ang_list = astar(start_point, goal_point, adjacency_list)  # Run A* to find a path and (v, ang) list
        
        # Draw path on the image
        image2 = image.copy()

        for i in range(len(path) - 1):
            cv2.line(image2, (int(path[i][0]*conversion), int((path[i][1])*conversion)), (int(path[i+1][0]*conversion), int((path[i+1][1])*conversion)), (255, 0, 0), thickness)

        image2 = cv2.resize(image2, (resizeX, resizeY))
        video_output.write(image2)
        add_blank_frames(image2,video_output,60,0.3)        

        # Identify obstacle nodes in the path
        obstacle_nodes = [node for node in path if inObstacle(node)]

        # Check if any obstacle nodes are still in the path
        if not any(inObstacle(node) for node in path):
            tf = time.time()
            print('Path Found in: ', tf-ti, 's')
            add_blank_frames(image2, video_output, 60, 3)         
            break  # If no obstacle nodes found in the path, exit the loop

        # Remove obstacle nodes from the adjacency list
        new_adjacency_list = {}
        for node, neighbors in adjacency_list.items():
            new_neighbors = [neighbor for neighbor in neighbors if neighbor[0] not in obstacle_nodes]
            new_adjacency_list[node] = new_neighbors

        adjacency_list = new_adjacency_list

    return path, v_ang_list, adjacency_list

radius = 22
clearance = 2
# Wheel speeds in RPM 
rpm1 = (int(40))
rpm2 = (int(80))
res = 1

spaceX = 600
spaceY = 200
map_array = np.zeros((spaceX, spaceY))
create_map()

# Visualization Size variables
visX = 6000 # pixels
visY = 2000 # pixels

# Used to scale node coordinates to visualization coordinates
conversion = visX//spaceX

# Resized canvas. We populate a 6000x2000 canvas to draw using the high resolution-
# Then we resize that drawn canvas to 1200x400-
# Technically we could have used 1200x400 from the start but it doesn't look as detailed.
resizeX = 1200 # pixels
resizeY = 500 # pixels
thickness = conversion

# DO NOT CHANGE - GAZEBO IS SET UP FOR THIS START NODE
start_point = (50,100,0)

######### CAN CHANGE - CAN PUT ANY VALUE BETWEEN X[0:600], Y[0:200], THETA[0:11] - (INTEGERS ONLY)#########
### IF A NODE IS IN AN OBSTACLE, THE CODE WILL FAIL ###
goal_point = (550, 100, 0)

# Example usage
num_samples = 600
space_limits = (0, spaceX, 0, spaceY)

# Samples map uniformly
ti = time.time()
sampled_points = sample_points_uniform(num_samples, space_limits, start_point, goal_point)
tf = time.time()
print('Sampled points in: ', tf-ti, 's')

# Initialize Canvas to visualization size
image = np.ones((visY, visX, 3), dtype=np.uint8) * 255  # White canvas

# Initialize video. Video is saved to working directory.
v_writer = cv2.VideoWriter_fourcc(*'mp4v')
fps = 60
video_output = cv2.VideoWriter('output.mp4', v_writer, fps, (resizeX, resizeY))

# Get obstacle coordinates
obstacles = obstacle_space()
# Initializes Map and Draws the obstacles
draw_obstacles(image, obstacles, video_output)

# Draw sampled points on the image
draw_points(image, sampled_points)
draw_nodes(image, start_point, goal_point)

# Create connections/Generate adjacency graph
ti = time.time()
adjacency_list = generate_adjacency_list(sampled_points)
tf = time.time()
print('Connected nodes in: ', tf-ti, 's')

# Draw connections on the image
draw_connections(image, adjacency_list)

# Run Lazy PRM algorithm
node_path, path, adjacency_list = run_lazy_prm(adjacency_list)
# print(path, '\n')
# print(node_path)
