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
        self.drone_width = 0.15
        self.offset_x = 0
        self.offset_y = 0
        self.yaw_range = np.pi/2
        self.yaw_scan_speed = 1
        self.horizon = 1
        self.mode = -1 # start at -1 for takeoff
        self.save_map = True
        self.startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]

        self.evade_dir = -1 # -1 to move right, 1 to move left
        self.path_count = 0
        self.threshold = 0
    # PID
        Kp, Ki, Kd = 2, 0, 0.5 # tune Kd bit to get a less-jumpy start on target switch
        Kp_z, Ki_z, Kd_z = 1, 5, 200 ##????
        vel_x, vel_y, vel_z, vel_yaw = 0.2, 0.2, 1, 1

        self.target_height = 1.0

        self.pid_x = PIDController(Kp, Ki, Kd, self.startpos[0], vel_x)
        self.pid_y = PIDController(Kp, Ki, Kd, self.startpos[1], vel_y)
        self.pid_z = PIDController(Kp_z, Ki_z, Kd_z, 0.5, vel_z) # for landing only
        self.pid_yaw = PIDController(Kp, Ki, Kd, 0, vel_yaw)
        # add one for height control

    # Maps
        self.min_x, self.max_x = 0, 5.0 # meter
        self.min_y, self.max_y = 0, 3.0 # meter
        self.range_max = 2.0 # meter, maximum range of distance sensor
        self.res_pos = 0.025 # meter old: 0.05
        self.conf = 0.2 # certainty given by each measurement
        self.t = 0 # only for plotting

        # Occupancy Map
        self.map = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos))) # 0 = unknown, 1 = free, -1 = occupied
    
        # Target Zone Map
        min_tar_x, max_tar_x = 3.5, 5
        self.map = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos))) # 0 unknown, -1 = occupied, 1 is checked)

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
        if self.mode == 1:
            self.generate_targetpath(sensor_data["y_global"], sensor_data)
        if self.mode == 2: # find target 
            return self.find_targetpad(sensor_data)
        if self.mode == 3: # land on target
            return self.land(sensor_data)
        if self.mode == 4:
            self.target_height = 1
            self.mode+=1
        if self.mode == 5:
            return self.take_off(sensor_data)
        if self.mode == 6:
            self.get_home_path()
        if self.mode == 7:
            return self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])
        return [0,0,0,0]

    def get_home_path(self):
        print("[LOG] Home path generated")
        self.mode+=1

    def take_off(self, sensor_data):
        control_command = self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])
        for i in range(len(control_command)):
            if i == 2:
                if abs(control_command[i]-sensor_data['range_down']) > 0.01:
                    return control_command
            elif abs(control_command[i]) > 0.01:
                return control_command
        print("[LOG] Launch successful")

        # set direction towards center
        if sensor_data['y_global'] > 2: # when starting further than 2 m left of map
            self.evade_dir = -1
        else:
            self.evade_dir = 1
        # print(self.evade_dir)

        self.mode += 1
        # self.mode = 2

        return control_command
    
    def generate_targetpath(self, pos_y, sensor_data):
        self.path = [(3.65, pos_y,"u"), (3.65, 2.8, "l"), (3.65, 0.2,"r"), (3.89, 0.2, "u"), 
                     (3.89, 2.8, "l"), (4.13, 2.8 , "u"), (4.13, 0.2,"r"), (4.37, 0.2, "u"), 
                     (4.37, 2.8, "l"), (4.61, 2.8, "u"), (4.61, 0.2, "r"), (4.85, 0.2, "u"),
                     (4.85, 2.8, "l")]
        
        print("[LOG] Target path generated")
        # print(self.path)
        self.mode += 1
        return self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])

    def find_targetpad(self, sensor_data):
        # return self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])

        pos_x, pos_y, yaw = sensor_data['x_global'], sensor_data['y_global'], sensor_data['yaw']
        cord_x, cord_y = self.p2c(pos_x), self.p2c(pos_y)
        tar_x, tar_y, direction = self.path[self.path_count]
        step_size = 0.2
        # self.drone_width = 0.1 #dont question it

        # print(tar_x, tar_y)
        # condition: abs(last height- current height) > 0.08, then target pad found
        # maybe add some center-finding of targetpad
        if abs(self.last_height - sensor_data['range_down']) > 0.08: # target pad height 0.1
            print("[LOG] Target pad found")
            self.mode += 1
            if direction == "l":
                self.pid_x.update_setpoint(pos_x)
                self.pid_y.update_setpoint(pos_y+0.05)
            if direction == "r":
                self.pid_x.update_setpoint(pos_x)
                self.pid_y.update_setpoint(pos_y-0.05)
            if direction == "u":
                self.pid_x.update_setpoint(pos_x+0.05)
                self.pid_y.update_setpoint(pos_y)
            print(self.pid_x.setpoint, self.pid_y.setpoint)
            self.pid_yaw.update_setpoint(0)
            return self.get_control(pos_x, pos_y, self.target_height, yaw)
        
        front_free = True
        left_free = True
        right_free = True
        back_free = True

        # print(pos_x, pos_y, tar_x, tar_y)

        if self.check_occupancy(cord_x+self.drone_map_width-self.offset_x, cord_y):
            front_free = False
        if self.check_occupancy(cord_x, cord_y+self.drone_map_width-self.offset_y):
            left_free = False
        if self.check_occupancy(cord_x, cord_y-self.drone_map_width+self.offset_y):
            right_free = False
        if self.check_occupancy(cord_x-self.drone_map_width+self.offset_x, cord_y):
            back_free = False
        
        # print(front_free, left_free, right_free, back_free)

        step_x = pos_x
        step_y = pos_y

        if direction == "r":
            if right_free and pos_x-tar_x > 0.05 and back_free and not self.check_occupancy(cord_x-self.drone_map_width, cord_y-self.drone_map_width): # can move forward but are next to path
                step_x = tar_x
            elif right_free and tar_x - pos_x > 0.05 and front_free and not self.check_occupancy(cord_x+self.drone_map_width, cord_y-self.drone_map_width):
                step_x = tar_x
            elif right_free and abs(pos_y-tar_y) > 0.1: # and not close to target
                    step_y -= step_size
            else:
                if abs(pos_x-tar_x) > 0.35:
                    if pos_x > tar_x: # too high
                        if back_free:
                            step_x -= step_size
                    else: # too low
                        if front_free:
                            step_x += step_size
                else:
                    if front_free and pos_x < 4.75 and abs(pos_y-tar_y) > 0.1:
                        step_x += step_size
                    elif back_free and abs(pos_y-tar_y) > 0.1:
                        step_x -= step_size
        
        if direction == "l":
            if left_free and pos_x-tar_x > 0.05 and back_free and not self.check_occupancy(cord_x-self.drone_map_width, cord_y+self.drone_map_width): # can move forward but are next to path
                step_x = tar_x
            elif left_free and tar_x - pos_x > 0.05 and front_free and not self.check_occupancy(cord_x+self.drone_map_width, cord_y+self.drone_map_width):
                step_x = tar_x
            elif left_free and abs(pos_y-tar_y) > 0.1: # and not close to target
                    step_y += step_size
            else:
                if abs(pos_x-tar_x) > 0.35:
                    if pos_x > tar_x: # too high
                        if back_free:
                            step_x -= step_size
                    else: # too low
                        if front_free:
                            step_x += step_size
                else:
                    if front_free and pos_x < 4.75 and abs(pos_y-tar_y) > 0.1:
                        step_x += step_size
                    elif back_free and abs(pos_y-tar_y) > 0.1:
                        step_x -= step_size

        if direction == "u":
            if front_free and abs(pos_x-tar_x) > 0.1: # and not close to target
                    step_x += step_size
            else:
                if abs(pos_y-tar_y) > 0.2: # too far left or right
                    if pos_y > tar_y:
                        if right_free:
                            step_y -= step_size
                    else:
                        if left_free:
                            step_y += step_size

        if step_x == pos_x and step_y == pos_y:
            self.path_count += 1 # target behind obstacle, go to next
            self.pid_x.update_setpoint(step_x) # add some if close to target croping
            self.pid_y.update_setpoint(step_y)
            self.last_height = sensor_data['range_down']
        
            return self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])

        # add some if close to target croping
        if direction == "r":
            # if step_x < tar_x:
            #     step_x = tar_x
            if step_y != pos_y and step_x == tar_x:
                step_y = tar_y
        
        if direction == "l":
            # if step_x > tar_x:
            #     step_x = tar_x
            if step_y != pos_y and step_x == tar_x:
                step_y = tar_y
        
        if direction == "u":
            if step_y > tar_y:
                step_y = tar_y
            if step_x != pos_x and step_y == tar_y:
                step_x = tar_x
        
        if abs(pos_x-tar_x) < 0.1 and abs(pos_y-tar_y) < 0.1:
            self.path_count += 1


        self.yaw_scan()
        self.pid_x.update_setpoint(step_x) 
        self.pid_y.update_setpoint(step_y)
        self.last_height = sensor_data['range_down']
        return self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])

    def land(self, sensor_data):

        self.target_height = 0.9
        if abs(sensor_data['x_global']-self.pid_x.setpoint) > 0.01 and abs(sensor_data['y_global']-self.pid_y.setpoint)> 0.01:
            return self.get_control(sensor_data['x_global'], sensor_data['y_global'], self.target_height, sensor_data['yaw'])
        
        # print("stabilized above target")
        if sensor_data['range_down']>0.5:
            comand_z = sensor_data['range_down']-0.1
        else:
            comand_z = sensor_data['range_down']-0.05
        control_command = self.get_control(sensor_data['x_global'], sensor_data['y_global'], comand_z, sensor_data['yaw'])
        # print(sensor_data['range_down'])
        
        if sensor_data['range_down'] < 0.015:
            print("[LOG] Landing successful")
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
            self.last_height = sensor_data['range_down']
            return self.get_control(pos_x, pos_y, self.target_height, yaw)
        
        dir_x = pos_x
        dir_y = pos_y

        cord_x = self.p2c(pos_x)
        cord_y = self.p2c(pos_y)

        left_free = True
        front_free = True
        right_free = True
        
        # Front
        if self.check_occupancy(cord_x+self.drone_map_width-self.offset_x, cord_y):
            front_free = False
        # Left side
        if self.check_occupancy(cord_x, cord_y+self.drone_map_width-self.offset_y) or self.check_occupancy(cord_x+self.drone_map_width-self.offset_x, cord_y+self.drone_map_width-self.offset_y):
            left_free = False
        # Right side
        if self.check_occupancy(cord_x, cord_y-self.drone_map_width+self.offset_y) or self.check_occupancy(cord_x+self.drone_map_width-self.offset_x, cord_y-self.drone_map_width+self.offset_y):
            right_free = False

        # print(left_free, front_free, right_free)

        if left_free and front_free and right_free:
            dir_x += step_size
            dir_y += (self.evade_dir*step_size)
        if left_free and front_free and not right_free:
            dir_x += step_size
            dir_y += step_size
        if left_free and not front_free and right_free:
            dir_y += (self.evade_dir*step_size)
        if left_free and not front_free and not right_free:
            dir_y += step_size
        if not left_free and front_free and right_free:
            dir_x += step_size
            dir_y -= step_size
        if not left_free and front_free and not right_free:
            dir_x += step_size
        if not left_free and not front_free and right_free:
            dir_y -= step_size
        if not left_free and not front_free and not right_free: # front obstacle clips left and right
            dir_x = pos_x - 0.1 # move back slightly
        #     if self.check_occupancy(cord_x, cord_y+self.drone_map_width-self.offset_y): # check if only left free
        #         dir_y += step_size
        #     if self.check_occupancy(cord_x, cord_y-self.drone_map_width+self.offset_y): # check if only right free
        #         dir_y -= step_size
        #     if dir_y == pos_y: # if both sides blocked, move evade directoin
        #         dir_x += (self.evade_dir*step_size)

        # add running_out_of_map_protection if trying to move e.g. front & right on border
        if dir_y < pos_y and pos_y < 0.1:
            dir_y = pos_y
        if dir_y > pos_y and pos_y > 2.9:
            dir_y = pos_y

        #update evade direction
        if pos_y > 2 and self.evade_dir == 1:
            self.evade_dir = -1
        if pos_y < 1 and self.evade_dir == -1:
            self.evade_dir = 1

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
                    if self.map[i][j] <= 0-self.threshold: # Cell occupied or unexplored
                        return True
                except:
                    return True # hit map border
        return False
    
    def p2c(self, x):
        return int(np.round(x/self.res_pos))
    
    def c2p(self, x):
        return x*self.res_pos + self.res_pos/2
