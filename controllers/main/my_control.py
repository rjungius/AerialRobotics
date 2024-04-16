# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2

# Global variables
controller = None

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
    global controller

    # Open a window to display the camera image
    # NOTE: Displaying the camera image will slow down the simulation, this is just for testing
    # cv2.imshow('Camera Feed', camera_data)
    # cv2.waitKey(1)
    
    # Take off
    # if startpos is None:
    #     startpos =    
    # if on_ground and sensor_data['range_down'] < 0.49:
    #     control_command = [0.0, 0.0, 1.0, 0.0]
    #     return control_command
    # else:
    #     if on_ground:
    #         on_ground = False

    # ---- YOUR CODE HERE ----
    
    if controller == None: 
        controller = Controller(sensor_data)

    return controller(sensor_data, dt)



## Cleaned code, also add class for graph/dijkstra for visibilty graph on way back

# Consider something that sets target to 1.5, 4.5, but the next distance X away. now if theres ojbect front, you move that by X back
# or left and right similarily (if right, add x, if object left, remove x -> if both sides object, we fine. if one side object, move left. 
# if we end up at our own position, we know its blocked front only and were in middle of map (otherwise I would move towards middle), and we need to step left)


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

class Controller:
    def __init__(self, sensor_data):
    # Other Constants
        self.drone_width = 0.2
        self.yaw_range = np.pi/2
        self.yaw_scan_speed = 1
        self.horizon = 1
        self.mode = -1 # start at -1 for takeoff
        self.save_map = True
        self.startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]

    # PID
        Kp, Ki, Kd = 2, 0, 0.2
        Kp_z, Ki_z, Kd_z = 1, 5, 200 ##????
        vel_x, vel_y, vel_z, vel_yaw = 0.3, 0.3, 1, 1

        self.target_height = 1.0

        self.pid_x = PIDController(Kp, Ki, Kd, self.startpos[0], vel_x)
        self.pid_y = PIDController(Kp, Ki, Kd, self.startpos[1], vel_y)
        self.pid_z = PIDController(Kp_z, Ki_z, Kd_z, 0.1, vel_z) # for landing only
        self.pid_yaw = PIDController(Kp, Ki, Kd, 0, vel_yaw)
        # add one for height control

    # Map
        self.min_x, self.max_x = 0, 5.0 # meter
        self.min_y, self.max_y = 0, 3.0 # meter
        self.range_max = 2.0 # meter, maximum range of distance sensor
        self.res_pos = 0.05 # meter
        self.conf = 0.2 # certainty given by each measurement
        self.t = 0 # only for plotting

        self.map = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos))) # 0 = unknown, 1 = free, -1 = occupied
    
    # Static calculations per Simulation
        self.drone_map_width = int(self.drone_width/self.res_pos)
        if self.drone_map_width % 2 == 0: # if even, make odd
            self.drone_map_width +=1

    def __call__(self, sensor_data, dt):
        return self.update_pid(sensor_data, dt)

    def update_pid(self, sensor_data, dt):
        self.occupancy_map(sensor_data)

        if self.mode == -1: #stabilize on startpad for testing:
            return self.take_off(sensor_data)
        if self.mode == 0:
            return self.reach_targetzone(sensor_data)
        # if self.mode == 1: # find target 
        if self.mode == 2: # land on target
            return self.land(sensor_data)
        return self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])

    def take_off(self, sensor_data):
        control_command = self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])
        for i in range(len(control_command)):
            if i == 2:
                if abs(control_command[i]-sensor_data['range_down']) > 0.01:
                    return control_command
            elif abs(control_command[i]) > 0.01:
                return control_command
        print("[LOG] Launch successful")
        # self.mode += 1
        self.mode = 2

        return control_command
    
    def land(self, sensor_data):
        control_command = self.get_control(sensor_data['x_global'], sensor_data['y_global'], 0, sensor_data['yaw'])
        control_command[2] = self.pid_z.update(sensor_data['range_down'])
        print(sensor_data['range_down'])
        if sensor_data['range_down'] < 0.015:
            print("[LOG] Landing successful")
            self.target_height = 0
            self.mode+=1
        return control_command

    def reach_targetzone(self, sensor_data): # task 1
        tar_x, tar_y = 4, 1.5
        step_size = 0.2
        pos_x, pos_y, yaw = sensor_data['x_global'], sensor_data['y_global'], sensor_data['yaw']
        
        if pos_x >= 3.5: 
            self.mode +=1
            print("[LOG] Target zone reached")
            self.pid_x.update_setpoint(pos_x)
            self.pid_y.update_setpoint(pos_y)
            self.pid_yaw.update_setpoint(0)
            return self.get_control(pos_x, pos_y, self.target_height, yaw)
        
        dir_x = min(pos_x + step_size, tar_x)
        dir_y = min(pos_y + step_size, max(pos_y - step_size, tar_y))

        cord_x = self.p2c(pos_x)
        cord_y = self.p2c(pos_y)

        if self.check_occupancy(cord_x+self.drone_map_width, cord_y):
            dir_x -= step_size
        if self.check_occupancy(cord_x, cord_y+self.drone_map_width) or self.check_occupancy(cord_x+self.drone_map_width, cord_y+self.drone_map_width):
            dir_y -= step_size
        if self.check_occupancy(cord_x, cord_y-self.drone_map_width) or self.check_occupancy(cord_x+self.drone_map_width, cord_y-self.drone_map_width):
            dir_y += step_size

        #if front blocked and both sides free:
        if abs(dir_x- pos_x) < 0.01 and abs(dir_y-pos_y) < 0.01: # with tolerance cuz why not
            if pos_y > 2.3: # when too far left
                dir_y -= step_size
            else:
                dir_y += step_size

        self.pid_x.update_setpoint(dir_x) #update x-target
        self.pid_y.update_setpoint(dir_y) # update y-target
        self.yaw_scan() # Wiggle

        return self.get_control(pos_x, pos_y, self.target_height, yaw)
    
    def yaw_scan(self):
        if self.pid_yaw.setpoint==0:
            self.pid_yaw.max_vel = self.yaw_scan_speed
            self.pid_yaw.update_setpoint(self.yaw_range)
        if abs(self.pid_yaw.prev_error) < 0.01:
            if self.pid_yaw.setpoint == self.yaw_range:
                self.pid_yaw.update_setpoint(-self.yaw_range)
            else:
                self.pid_yaw.update_setpoint(self.yaw_range)

    def get_control(self, pos_x, pos_y, height, yaw): 
        control_command = [0.0,0.0,height, 0.0]
        d_x = self.pid_x.update(pos_x)
        d_y = self.pid_y.update(pos_y)

        control_command[0] = d_x * np.cos(yaw) + d_y * np.sin(yaw)
        control_command[1] = -d_x * np.sin(yaw) + d_y * np.cos(yaw)
        # control_command[2] = self.target_height + max(0, self.pid_z.update(height)) # should work
        control_command[2] = height # should work
        control_command[3] = self.pid_yaw.update(yaw)

        return control_command
    
    def occupancy_map(self, sensor_data):
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

            for i in range(int(self.range_max/self.res_pos)): # range is 2 meters
                dist = i*self.res_pos
                idx_x = int(np.round((pos_x - self.min_x + dist*np.cos(yaw_sensor))/self.res_pos,0))
                idx_y = int(np.round((pos_y - self.min_y + dist*np.sin(yaw_sensor))/self.res_pos,0))

                # make sure the current_setpoint is within the map
                if idx_x < 0 or idx_x >= self.map.shape[0] or idx_y < 0 or idx_y >= self.map.shape[1] or dist > self.range_max:
                    break

                # update the map
                if dist < measurement:
                    self.map[idx_x, idx_y] += self.conf
                else:
                    self.map[idx_x, idx_y] -= self.conf
                    break
                
        self.map = np.clip(self.map, -1, 1) # certainty can never be more than 100%

        # only plot every Nth time step (comment out if not needed)
        if self.t % 25 == 0 and self.save_map:
            plt.imshow(np.flip(self.map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
            plt.savefig("map.png")
            plt.close()
        self.t +=1

        return map

    def check_occupancy(self, c_x, c_y):
        for i in range(c_x - int(self.drone_map_width/2), c_x + int(self.drone_map_width/2)+1):
            for j in range(c_y - int(self.drone_map_width/2), c_y + int(self.drone_map_width/2)+1):
                try:
                    if self.map[i][j] <= 0: # Cell occupied or unexplored
                        return True
                except:
                    return True # hit map border
        return False
    
    def p2c(self, x):
        return int(np.round(x/self.res_pos))
    
    def c2p(self, x):
        return x*self.res_pos + self.res_pos/2
