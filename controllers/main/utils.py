import numpy as np

# Create a rotation matrix from the world frame to the body frame using euler angles
def euler2rotmat(euler_angles):
    
    R = np.eye(3)
    
    # Here you need to implement the rotation matrix
    # First calculate the rotation matrix for each angle (roll, pitch, yaw)
    # Then multiply the matrices together to get the total rotation matrix

    # Inputs:
    #           euler_angles: A list of 3 Euler angles [roll, pitch, yaw] in radians
    # Outputs:
    #           R: A 3x3 numpy array that represents the rotation matrix of the euler angles
    
    # --- YOUR CODE HERE ---
    phi = euler_angles[0] # roll
    theta = euler_angles[1] # pitch
    psi = euler_angles[2] # yaw

    R_roll = np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]])
    R_pitch = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    R_yaw = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])

    R = R_yaw*R_pitch*R_roll
    
    return R

# Rotate the control commands from the body reference frame to the inertial reference frame
def rot_body2inertial(control_commands, euler_angles):
    
    # Here you need to rotate the control commands from the body reference frame to the inertial reference frame
    # You should use the euler2rotmat function to get the rotation matrix
    # Keep in mind that you only want to rotate the velocity commands, which are the first two elements of the control_commands array
    # Think carefully about which direction you need to perform the rotation

    # Inputs:
    #           control_commands: A list of 4 control commands [vel_x, vel_y, altitude, yaw_rate] in the body reference frame
    #           euler_angles: A list of 3 Euler angles [roll, pitch, yaw] in radians
    # Outputs:
    #           control_commands: A list of 4 control commands [vel_x, vel_y, altitude, yaw_rate] in the inertial reference frame

    # --- YOUR CODE HERE ---

    # Controller is in World-frame
    vel_body = np.array([control_commands[0], control_commands[1], 0])
    
    R = euler2rotmat(euler_angles)
    
    vel_world = np.matmul(R,np.transpose(vel_body))
    print(R)
    # np.transpose()
    print(vel_world)
    print(vel_body)
    control_commands[0] = vel_world[0]

    control_commands[1] = vel_world[1]


    return control_commands

   
