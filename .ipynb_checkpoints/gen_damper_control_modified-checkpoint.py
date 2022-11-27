import numpy as np

def  gen_damper_control_modified(R_w_EE, F_z, F_x, tau, b_z, b_x, b_omega, d_FT_COC, d_EE_COC, F_x_des):
    '''
    Control motion of a robot in a plane with a generalized damper
    Alex Elias

    Assumption:
    end-effector frame, force/torque sensor frame,
    and center of compliance frame are all the same
    except for an offset in their (local) Z location

    Inputs:
        theta: rotation of end effector
        F_z, F_x, tau: forces on F/T sensor (in the F/T sensor frame)
        b_z, b_x, b_omega: damper parameters
        d_FT_COC: displacement in Z direction from FT sensor to center of compliance
        d_EE_COC: displacement in Z direction from end-effector frame to center of compliance
        F_x_des: desired force in Z direction
    
    Outputs:
        v_x, v_z, omega: Velocities of end effector in the world frame
    '''

    # Find force and torque in the COC frame
    # COC frame is same as FT frame, but translated by d in the FT Z direction
    F_z_COC = F_z
    F_x_COC = F_x
    tau_COC = tau + d_FT_COC * F_z

    # Compute velocities of COC frame in COC frame
    v_z_COC = F_z_COC / b_z
    v_x_COC = (F_x_COC + F_x_des) / b_x
    omega_COC = tau_COC / b_omega

    # Convert to velocities of the end effector in the world frame

    # Velocity of end effector in COC frame
    v_x_EE_COC = v_x_COC 
    v_z_EE_COC = v_z_COC + d_EE_COC * omega_COC
    omega_EE_COC = omega_COC

    # And now rotate to world frame
    
    # COC and EE has same frame
    v_EE = R_w_EE @ np.array([v_x_EE_COC,0, v_z_EE_COC]).T
    omega_EE = omega_EE_COC

    return v_EE, omega_EE

def test_gen_damper_control():
    R_w_EE = np.eye(3)
    F_z = 0.0
    F_x = 0.0
    tau = 0.0
    b_z = 1.0
    b_x = 1.0
    b_omega = 1.0
    d_FT_COC = 0.2
    d_EE_COC = 0.1
    F_x_des = 0.0

    v_EE, omega_EE = gen_damper_control_modified(R_w_EE, F_z, F_x, tau, b_z, b_x, b_omega, d_FT_COC, d_EE_COC, F_x_des)
    print(v_EE, omega_EE)
    
    theta = -np.pi/4
    R_w_EE = np.array([[np.cos(theta),0,np.sin(theta)],
                       [0,1,0],
                       [-np.sin(theta),0,np.cos(theta)]])
    F_x_des = 10.0
    F_x = 10.0
    F_z = 0
    v_EE, omega_EE = gen_damper_control_modified(R_w_EE, F_z, F_x, tau, b_z, b_x, b_omega, d_FT_COC, d_EE_COC, F_x_des)
    print(v_EE, omega_EE)

if __name__ == '__main__':
    test_gen_damper_control()