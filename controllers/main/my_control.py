# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import networkx as nx

# Global variables
on_ground = True
height_desired = 1.0
timer = None
startpos = None
timer_done = None
mode = None
graph = None
evade = None

# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py lines 296-323. 
# The "item" values that you can later use in the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "range_down": Downward range finder distance (Used instead of Global Z distance)
# "range_front": Front range finder distance
# "range_left": Leftward range finder distance 
# "range_right": Rightward range finder distance
# "range_back": Backward range finder distance
# "yaw": Yaw angle (rad)

# This is the main function where you will implement your control algorithm
def get_command(sensor_data, camera_data, dt):
    global on_ground, startpos, mode, graph, evade

    # Open a window to display the camera image
    # NOTE: Displaying the camera image will slow down the simulation, this is just for testing
    # cv2.imshow('Camera Feed', camera_data)
    # cv2.waitKey(1)
    
    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]   
        print(startpos)
        print(int(startpos[0]/res_pos), int(startpos[1]/res_pos), int(startpos[2]/res_pos))
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, height_desired, 0.0]
        return control_command
    else:
        if on_ground:
            print(sensor_data['x_global'], sensor_data['y_global'])
            print(int(sensor_data['x_global']/res_pos), int(sensor_data['y_global']/res_pos))
        on_ground = False

    # ---- YOUR CODE HERE ----
    if mode == None: 
        mode = 0
    if graph == None:
        graph = nx.grid_2d_graph(int(5/res_pos), int(3/res_pos))
        for u,v in graph.edges():
            graph[u][v]["weight"] = 1
    
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    idx_x = int(np.round((pos_x)/res_pos,0))
    idx_y = int(np.round((pos_y)/res_pos,0))
    map = occupancy_map(sensor_data)
    
    if mode == 0: # Reach target area
        control_command = [0.0, 0.0, height_desired, 0]
        
        if pos_x>3.5:
            print("Landing Zone reached")
            mode = 1
            return control_command
        
        if evade == None:
            if sensor_data['range_front'] > 0.3 and map[idx_x+1, idx_y]:
                control_command[0]=0.3
            else: #decide to turn either left or right, and then evade until front is free
                if pos_y < 1.5 and sensor_data["range_left"] > 0.4:
                    evade = "left"
                else:
                    if sensor_data["range_right"] > 0.4:
                        evade = "right"
                    else:
                        evade = "left"
        
        if sensor_data["range_left"] < 0.2:
            control_command[1] = -0.3
        if sensor_data["range_right"] < 0.2:
            control_command[1] = 0.3

        if evade == "left":
            control_command[1] = 0.3
            if sensor_data["range_front"] > 0.4:
                evade = None
        if evade == "right":
            control_command[1] = -0.3
            if sensor_data["range_front"] > 0.4:
                evade = None

    elif mode == 1: # Find target pad
        control_command = [0.0, 0.0, height_desired, 0.0]
    elif mode == 2: # Land on target pad
        control_command = [0.0, 0.0, height_desired, 0.0]
    elif mode == 3: # Leave landing pad
        control_command = [0.0, 0.0, height_desired, 0.0]
    elif mode == 4: # Fly to start pad
        control_command = [0.0, 0.0, height_desired, 0.0]
    elif mode == 5: # Land on start pad
        control_command = [0.0, 0.0, height_desired, 0.0]

    return control_command # Ordered as array with: [v_forward_cmd, v_left_cmd, alt_cmd, yaw_rate_cmd]


# Occupancy map based on distance sensor
min_x, max_x = 0, 5.0 # meter
# min_y, max_y = 0, 5.0 # meter ### only 3 m in width?
min_y, max_y = 0, 3.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.2 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range_front']
        elif j == 1:
            measurement = sensor_data['range_left']
        elif j == 2:
            measurement = sensor_data['range_back']
        elif j == 3:
            measurement = sensor_data['range_right']
        
        for i in range(int(range_max/res_pos)): # range is 2 meters
            dist = i*res_pos
            idx_x = int(np.round((pos_x - min_x + dist*np.cos(yaw_sensor))/res_pos,0))
            idx_y = int(np.round((pos_y - min_y + dist*np.sin(yaw_sensor))/res_pos,0))

            # make sure the current_setpoint is within the map
            if idx_x < 0 or idx_x >= map.shape[0] or idx_y < 0 or idx_y >= map.shape[1] or dist > range_max:
                break

            # update the map
            if dist < measurement:
                map[idx_x, idx_y] += conf
            else:
                map[idx_x, idx_y] -= conf
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    # only plot every Nth time step (comment out if not needed)
    if t % 150 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        plt.savefig("map.png")
        plt.close()
    t +=1

    return map


def update_graph(graph, map):
    x,y = map.shape
    for i in range(x):
        for j in range(y):
            if map[i][j] < -0.3 and graph.has_node((i,j)):
                graph.remove_node((i,j))





































# Control from the exercises
index_current_setpoint = 0
def path_to_setpoint(path,sensor_data,dt):
    global on_ground, height_desired, index_current_setpoint, timer, timer_done, startpos

    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]    
    if on_ground and sensor_data['range_down'] < 0.49:
        current_setpoint = [startpos[0], startpos[1], height_desired, 0.0]
        return current_setpoint
    else:
        on_ground = False

    # Start timer
    if (index_current_setpoint == 1) & (timer is None):
        timer = 0
        print("Time recording started")
    if timer is not None:
        timer += dt
    # Hover at the final setpoint
    if index_current_setpoint == len(path):
        # Uncomment for KF
        control_command = [startpos[0], startpos[1], startpos[2]-0.05, 0.0]

        if timer_done is None:
            timer_done = True
            print("Path planing took " + str(np.round(timer,1)) + " [s]")
        return control_command

    # Get the goal position and drone position
    current_setpoint = path[index_current_setpoint]
    x_drone, y_drone, z_drone, yaw_drone = sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down'], sensor_data['yaw']
    distance_drone_to_goal = np.linalg.norm([current_setpoint[0] - x_drone, current_setpoint[1] - y_drone, current_setpoint[2] - z_drone, clip_angle(current_setpoint[3]) - clip_angle(yaw_drone)])

    # When the drone reaches the goal setpoint, e.g., distance < 0.1m
    if distance_drone_to_goal < 0.1:
        # Select the next setpoint as the goal position
        index_current_setpoint += 1
        # Hover at the final setpoint
        if index_current_setpoint == len(path):
            current_setpoint = [0.0, 0.0, height_desired, 0.0]
            return current_setpoint

    return current_setpoint

def clip_angle(angle):
    angle = angle%(2*np.pi)
    if angle > np.pi:
        angle -= 2*np.pi
    if angle < -np.pi:
        angle += 2*np.pi
    return angle