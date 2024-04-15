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
next_cell = None
pid_vel_x = None
pid_vel_y = None
pid_z = None
pid_yaw = None

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
    global on_ground, startpos, mode, graph, next_cell, pid_vel_x, pid_vel_y, pid_z, pid_yaw

    # Open a window to display the camera image
    # NOTE: Displaying the camera image will slow down the simulation, this is just for testing
    # cv2.imshow('Camera Feed', camera_data)
    # cv2.waitKey(1)
    
    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]   
        print(f'startpos: {startpos}')
        print(int(np.round(startpos[0]/res_pos)), int(np.round(startpos[1]/res_pos)))
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, height_desired, 0.0]
        return control_command
    else:
        if on_ground:
            print(sensor_data['x_global'], sensor_data['y_global'])
            print(int(sensor_data['x_global']/res_pos), int(sensor_data['y_global']/res_pos))
            on_ground = False

    # ---- YOUR CODE HERE ----
    pos_to_cord = lambda x: int(np.round(x/res_pos))
    cord_to_pos = lambda x: x*res_pos + res_pos/2
 
    if mode == None: 
        mode = -1
        Kp, Ki, Kd = 2, 0, 0.2
        pid_vel_x = PIDController(Kp, Ki, Kd, startpos[0], 0.3)
        pid_vel_y = PIDController(Kp, Ki, Kd, startpos[1], 0.3)
        pid_yaw = PIDController(Kp, Ki, Kd, 0, 0.7)
    if graph == None:
        graph = nx.grid_2d_graph(int(5/res_pos), int(3/res_pos))
        for u,v in graph.edges():
            graph[u][v]["weight"] = 1
    
    # print(dt)

    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    idx_x = pos_to_cord(pos_x)
    idx_y = pos_to_cord(pos_y)
    map = occupancy_map(sensor_data)
    
    control_command = [0, 0, height_desired, 0]
    
    if mode == -1: #stabilize on startpad for testing:
        # print(startpos[0]-pos_x, startpos[1]-pos_y, yaw)
        
        control_command[0] = pid_vel_x.update(pos_x)
        control_command[1] = pid_vel_y.update(pos_y)
        control_command[2] = height_desired
        control_command[3] = pid_yaw.update(yaw)

        mode = 0
        for i in control_command:
            if i == height_desired:
                continue
            if i > 0.001:
                mode = -1
        if mode == 0:
            print("start pos reached")
        return control_command
    
    if mode == 0: # Reach target area
        tar_x = 4
        tar_y = 1.5
        tol = 0.01
        yaw_scan = 1

        # pid_yaw.update_setpoint(0)
        pid_vel_x.update_setpoint(min(pos_x+sensor_data['range_front']-0.2, tar_x))

        if check_occupancy(map, pos_to_cord(pos_x), pos_to_cord(pos_y), "front", 0.2, 4):
            print("front path blocked")
            pid_vel_x.update_setpoint(pos_x-0.1)
        else:
            print("front path free")
            # print(pid_vel_x.setpoint)


        tmp_y = pos_y # try to go towards mid of map unless object
        if sensor_data['range_left'] < 0.2:
            tmp_y -= (0.2 - sensor_data["range_left"])
        elif sensor_data['range_right'] < 0.2:
            tmp_y += (0.2 - sensor_data["range_right"])
        else:
            tmp_y = tar_y
        pid_vel_y.update_setpoint(tmp_y)
        
        if check_occupancy(map, pos_to_cord(pos_x), pos_to_cord(pos_y), "right", 0.2, 4):
            print("right path blocked")
            pid_vel_x.update_setpoint(pos_y+0.2)
        else:
            print("right path free")
        if check_occupancy(map, pos_to_cord(pos_x), pos_to_cord(pos_y), "left", 0.2, 4):
            print("left path blocked")
            pid_vel_x.update_setpoint(pos_y-0.2)
        else:
            print("left path free")
        print("\n")

        # some yaw-trying-stuff for scanning
        # print(pid_yaw.setpoint, pid_yaw.prev_error)
        if pid_yaw.setpoint==0:
            pid_yaw.max_vel = 0.3
            pid_yaw.update_setpoint(yaw_scan)
        if abs(pid_yaw.prev_error) < 0.01:
            if pid_yaw.setpoint == yaw_scan:
                pid_yaw.update_setpoint(-yaw_scan)
            else:
                pid_yaw.update_setpoint(yaw_scan)
        

        d_x = pid_vel_x.update(pos_x)
        d_y = pid_vel_y.update(pos_y)

        control_command[0] = d_x * np.cos(yaw) + d_y * np.sin(yaw)
        control_command[1] = -d_x * np.sin(yaw) + d_y * np.cos(yaw)
        control_command[2] = height_desired
        control_command[3] = pid_yaw.update(yaw)

        return control_command

        if pos_x>3.5:
            print("Landing Zone reached")
            mode = 1
            return control_command
        
        if next_cell == None:
            update_graph(graph, map)
            target = (pos_to_cord(tar_x), pos_to_cord(tar_y))
            path = nx.astar_path(graph, (idx_x, idx_y), target)
            next_cell = path[1]
            # print(path)
            # next_cell = (pos_to_cord(startpos[0])+2, pos_to_cord(startpos[1]))

            pid_vel_x.update_setpoint(cord_to_pos(next_cell[0]))
            pid_vel_y.update_setpoint(cord_to_pos(next_cell[1]))
            print(f'next target: {next_cell}')

        control_command[0] = pid_vel_x.update(pos_x)
        control_command[1] = pid_vel_y.update(pos_y)
        control_command[3] = pid_yaw.update(yaw)
        if abs(control_command[0]) < tol and abs(control_command[1]) < tol:
            next_cell = None

        if sensor_data["range_front"] < 0.2: # local avoidance
            control_command[0] = -0.2
            next_cell = None

        if sensor_data["range_left"] < 0.2:
            control_command[1] = -0.2
            next_cell = None

        if sensor_data["range_right"] < 0.2:
            control_command[1] = 0.2
            next_cell = None

        # print(control_command)

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
res_pos = 0.1 # meter
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

def check_occupancy(map, coord_x, coord_y, direction, drone_width, search_horizon):
    check_breadth = int(drone_width/2/res_pos)
    
    if direction == "left":
        for i in range(coord_x-check_breadth, coord_y+check_breadth+1):
            for j in range(coord_y+check_breadth +1, coord_y+search_horizon*check_breadth):
                try:
                    if map[i][j] < 0: #if unknown, wait for further data by wiggeling
                        return True
                except:
                    continue #field out of bounds so free
    
    if direction == "front":
        for i in range(coord_x+check_breadth+1, coord_x+search_horizon*check_breadth):
            for j in range(coord_y-check_breadth, coord_y+check_breadth+1):
                try:
                    if map[i][j] < 0: #if unknown, wait for further data by wiggeling
                        return True
                except:
                    continue #field out of bounds so free
    
    if direction == "right":
        for i in range(coord_x-check_breadth, coord_y+check_breadth+1):
            for j in range(coord_y-search_horizon*check_breadth, coord_y-check_breadth):
                try:
                    if map[i][j] < 0: #if unknown, wait for further data by wiggeling
                        return True
                except:
                    continue #field out of bounds so free
    return False

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, max_vel):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.max_vel = max_vel
        
        self.prev_error = 0
        self.integral = 0
        
    def update(self, feedback_value):
        error = self.setpoint - feedback_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral
        
        # Derivative term
        D = self.Kd * (error - self.prev_error)
        
        # PID output
        output = P + I + D
        # print(output)
        
        # Update previous error
        self.prev_error = error
        
        return max(min(output, self.max_vel), -self.max_vel)

    def update_setpoint(self, setpoint):
        self.setpoint = setpoint

































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