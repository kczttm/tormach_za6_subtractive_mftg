from manual_guiding import *


def z_touch_off(m_rob):
    R_home = m_rob.home_H.R
    with open('config/tool_pose_modified.yaml') as tool_file:
        H_tool=np.array(yaml.safe_load(tool_file)['H'], dtype=np.float64)    
    R_tool = H_tool[:3,:3]
    d_FT_COC = 0.0229  # meter, COC -> center of compliance
    F_z_des = 1.7  # N in COC frame
    count = 0
    touchpoint = None
    ###enable velocity emulation
    m_rob.vel_ctrl.enable_velocity_mode() 
    while True:
        try:
            #convert tool frame to base frame
            q_cur=m_rob.vel_ctrl.joint_position()
            R=m_rob.robot_toolbox.fwd(q_cur).R
            if np.linalg.norm(m_rob.torque)>1 or np.linalg.norm(m_rob.force)>1:
                torque_EE=np.dot(R_tool,m_rob.torque)
                force_EE=np.dot(R_tool,m_rob.force)
            else:
                torque_EE=np.zeros((3,1))
                force_EE=np.zeros((3,1))

            ####### compliance in z dir
            force_z = force_EE[-1][0]
            torque_y = torque_EE[1][0]
            F_z_COC = force_z + (-d_FT_COC)*torque_y
            ###dynamic damping coefficient
            damping_coeff=max(np.linalg.norm(np.array([force_z, torque_y]))/1.5,1) #/2
            b_z = 0.2*damping_coeff
            ### v from COC frame to base frame
            v_z_COC = (F_z_COC - F_z_des) / b_z
            # COC and EE has same frame
            R_bf_EE = R @ R_tool.T  # R_wd_
            v_bf = R_bf_EE @ np.array([0,0, v_z_COC])
            #print('damping', damping_coeff*2000.)
            ###pass to controller

            ###QP controller, cartesian linear motion and orientation
            vd=np.round(np.clip(v_bf, -100, 100),3)  # in mm somehow
            count += 1
            if count % 100 == 0:
                print('vd', vd,
                      'force_EE:', np.round(force_EE.T[0],3),damping_coeff)
            m_rob.move(vd, np.dot(R, R_home.T))
            #######
            if np.linalg.norm(vd) < 0.06:
                touchpoint = m_rob.robot_toolbox.fwd(q_cur).p
                m_rob.jog_joint_movel(touchpoint+np.array([0,0,10]), 10,threshold=0.05, acc_range=0., Rd = R_home)
                break
        except:
            traceback.print_exc()
            break

    m_rob.vel_ctrl.set_velocity_command(np.zeros(6))
    m_rob.vel_ctrl.disable_velocity_mode()
    return touchpoint

if __name__ == "__main__":
    manual_loc = manual_guiding()
    manual_loc.start(homing = True)
    time.sleep(15)
    manual_loc.stop()
    p1 = z_touch_off(manual_loc)
    print(p1)