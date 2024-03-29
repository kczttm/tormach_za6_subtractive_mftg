from RobotRaconteur.Client import *
import numpy as np
np.set_printoptions(suppress = True)
import matplotlib.pyplot as plt

import time, sys, yaml, os, threading, traceback
from importlib import import_module
from general_robotics_toolbox import *

import traj_gen_curve
import traj_gen_wave
sys.path.append('toolbox')
from lambda_calc import *
from vel_emulate_sub import EmulatedVelocityControl
from qpsolvers import solve_qp
from robots_def import *
from realtime_joint_measure import Joint_Realtime
import Spindle as SC

class tormach_machining():
    '''
    Desired trajectories are splited into segments
    ATI force torque sensor is enabled to check for contact before each segments
    The excecutability of the path is analyzed before the exicution
    
    '''
    def __init__(self):

        self.torque=np.zeros(3)
        self.force=np.zeros(3)
        
        self.vd = 10  # mm/s
        self.cutting_depth = 0  # mm
        self.data_path = None
        self.data_name = None

        with open('config/tormach_za06_robot_default_config.yml') as robot_file:
            with open('config/ati_pose.yaml') as tool_file:
                self.robot_toolbox=yml2robdef(robot_file,tool_file)
        ################################Connect to FT Sensor###################################
        with open('config/workspace_H.yaml') as f:
            self.H_base_curve = np.array(yaml.safe_load(f),dtype=np.float64)
            
        url='rr+tcp://localhost:59823?service=ati_sensor'
        if (len(sys.argv)>=2):
            url=sys.argv[1]

        #Connect to the service
        cli = RRN.ConnectService(url)

        #Connect a wire connection
        wrench_wire = cli.wrench_sensor_value.Connect()

        #Add callback for when the wire value change
        wrench_wire.WireValueChanged += self.wrench_wire_cb

        ##########################Connect Robot Service###########################################
        tormach_url='rr+tcp://[fe80::180d:c0c1:b05b:57f7]:11111/?nodeid=b1141357-a2c8-41dc-b16d-02dc9610ddea&service=tormach_robot'
    #	robot_sub=RRN.SubscribeService('rr+tcp://pi-tormach:11111?service=tormach_robot')
        robot_sub=RRN.SubscribeService(tormach_url)
        self.robot=robot_sub.GetDefaultClientWait(1)
        state_w = robot_sub.SubscribeWire("robot_state")
        cmd_w = robot_sub.SubscribeWire("position_command")
        RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",self.robot)
        self.vel_ctrl = EmulatedVelocityControl(self.robot,state_w, cmd_w)

        robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", self.robot)
        state_flags_enum = robot_const['RobotStateFlags']
        self.halt_mode = robot_const["RobotCommandMode"]["halt"]
        self.position_mode = robot_const["RobotCommandMode"]["position_command"]
        self.trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
        #jog_mode = robot_const["RobotCommandMode"]["jog"]

        print(self.robot.robot_info.device_info.device.name+" Connected")
        
        ######### Connect to spindle############
        self.spindle = SC.Controller(self.robot)
        self.spindle.Stop()
        
        
        if os.path.exists('home_joint_angles.npy'):
            with open('home_joint_angles.npy', 'rb') as f:
                q_top = np.load(f)
            print('Found and loaded home joint config:\n',
                  q_top)
        
        self.home_H = self.robot_toolbox.fwd(q_top)
        self.home_q = q_top
        
    def load_traj(self, data_path = 'data/wave/',data_name='Curve'):
        self.data_path = data_path
        self.data_name = data_name
        curve_bf = read_csv(data_path+data_name+'_in_base_frame.csv',header=None).values
        lam=calc_lam_cs(curve_bf)

        lam_diff=np.gradient(lam)
        dt=lam_diff/self.vd
        t_traj=np.cumsum(dt)
        return curve_bf, t_traj
        
        
    def eval_traj(self, curve_bf, t_traj):
        plt.plot(t_traj,curve_bf[:,:3])
        plt.title('tool tip trajectorys')
        plt.xlabel('time (sec)')
        plt.ylabel('milimeter')
        plt.legend(['x','y','z'])
        plt.xlim([t_traj[0]-0.1, t_traj[-1]+0.1]) 
        plt.show()

        ## evaluate angle limit
        curve_js = read_csv(self.data_path+self.data_name+'_js.csv',header=None).values
        fig, axs = plt.subplots(6,1, figsize = (9, 12))
        joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        for idx, ax in enumerate(axs):
            ax.hlines(np.array([self.robot_toolbox.lower_limit[idx], 
                                self.robot_toolbox.upper_limit[idx]])*180/np.pi, 
                      t_traj[0], t_traj[-1], 
                      linestyles='dashed', color='red',linewidth=4)
            ax.plot(t_traj, curve_js[:,idx]*180/np.pi)
            ax.set_ylabel(joint_names[idx])

        ## evaluate jacobian for singularities
        svd_J = []
        for q_calc in curve_js:
            robot_J = self.robot_toolbox.jacobian(q_calc)
            u,s,vh = np.linalg.svd(robot_J)
            svd_J.append(s)
        svd_J = np.array(svd_J)
        fig2, ax2 = plt.subplots()
        plt.plot(t_traj,np.min(svd_J, axis = 1))
        plt.title('trajectory jacobian smallest singular values')
        plt.xlabel('time (sec)')
        plt.ylabel('sigma')
        #plt.legend(traj.joint_names)
        plt.show()
        
        
    def jog_traj(self, curve_bf, t_traj, depth = -10, spindle = False, rpm = 15000, validation = False):
        ## check if ATI sensor is on
        if np.linalg.norm(self.force)==0 and np.linalg.norm(self.force)==0 : 
            print('Restart ATI sensor driver')
            return
        
        ## first jog to the starting point of the trajectory
        self.robot.command_mode = self.halt_mode
        time.sleep(0.1)
        self.robot.command_mode = self.position_mode
        ## enable velocity emulation
        self.vel_ctrl.enable_velocity_mode()
        print('velocity control enabled')
        if validation:
            ## jog the robot to above starting point
            p_up_bf = self.H_base_curve[:3,:3] @ np.array([0,0,10])
            self.jog_joint_movel(curve_bf[0,:3]+p_up_bf, 
                                 max_v = 20,threshold=0.04, 
                                 acc_range=0., Rd = self.home_H.R)
            ## refind the distance to the surface
            curr_point = self.z_touch(lift = True) 
            off_set = curr_point - curve_bf[0,:3]
            print('off_set of ', off_set, ' will be applied to the current trajectory')
        else:
            self.jog_joint_movel(curve_bf[0,:3], 
                                 max_v = 20,threshold=0.04, 
                                 acc_range=0., Rd = self.home_H.R)
            off_set = np.zeros(3)
            
        if spindle:
            self.cutting_depth = depth
            self.spindle.Start()
            if self.spindle.setSpeed(rpm):
                print('Spindle Speed at ', rpm)   
        print('drilling down ', depth, ' mm')
        p_down = self.H_base_curve[:3,:3] @ np.array([0,0,-depth])
        print('offset is ', '\n', off_set)
        self.jog_joint_movel(curve_bf[0,:3]+off_set+p_down, 
                             max_v = 5,threshold=0.04, 
                             acc_range=0., Rd = self.home_H.R)
        time.sleep(2)
        # self.spindle.Stop()
        # self.vel_ctrl.set_velocity_command(np.zeros(6))
        # self.vel_ctrl.disable_velocity_mode()
        # return
        print('trajectory starting')
        ### jog robot through the way points according to time stamp
        threshold=0.2
        acc_range=0.01
        dcc_range=0.04
        Rd=self.home_H.R
        
        pt_id = 1  # which position to hit
        pt_last = len(t_traj)  # last point to hit will have decceleration
        ## start pose recorder:
        rt_joint = Joint_Realtime(self.vel_ctrl)
        rt_joint.start()
        while pt_id < pt_last: 
            ## get starting pos of this segment
            p = curve_bf[pt_id,:3]+off_set+p_down  # target pose
            q_cur=self.vel_ctrl.joint_position()
            p_init=self.robot_toolbox.fwd(q_cur).p
            diff_targ = p - curve_bf[pt_id-1,:3]
            diff_targ_norm = np.linalg.norm(diff_targ)
            
            t_targ = t_traj[pt_id]  # target time of arrival
            dt = t_targ - t_traj[pt_id-1]
            # v_avg_mag = min(diff_targ_norm/dt, max_v)  # nominal path speed <= max_v
            v_avg_mag = min(diff_targ_norm/dt, self.vd)
            v_avg = v_avg_mag*diff_targ/diff_targ_norm,self.vd
            # print(pt_id, dt, diff_norm)
            diff_norm = diff_targ_norm
            dynamic_threshold = v_avg_mag * 0.01  # assume data acquisition period = 0.01s
            while diff_norm>dynamic_threshold:
                q_cur=self.vel_ctrl.joint_position()
                pose_cur=self.robot_toolbox.fwd(q_cur)
                diff=p-pose_cur.p
                diff2=pose_cur.p-p_init

                diff_norm=np.linalg.norm(diff)
                diff2_norm=np.linalg.norm(diff2)
                
                v = v_avg_mag*diff/diff_norm

                ###correcting orientation
                self.move(v,np.dot(pose_cur.R,Rd.T))
                # print(pt_id,diff_norm)
            pt_id +=1
        rt_joint.stop()
        
        q_cur=self.vel_ctrl.joint_position()
        pose_cur_bf=self.robot_toolbox.fwd(q_cur).p
        p_up_bf = self.H_base_curve[:3,:3] @ np.array([0,0,10])
        self.jog_joint_movel(pose_cur_bf + p_up_bf,
                                         10,threshold=0.05, 
                                         acc_range=0., Rd = Rd)
        if spindle:
            self.spindle.Stop()
        
        self.vel_ctrl.set_velocity_command(np.zeros(6))
        self.vel_ctrl.disable_velocity_mode()
        print('velocity control disabled')
        
        end_pose_real = []
        for q_real in rt_joint.joint_position:
            robot_pose = self.robot_toolbox.fwd(q_real)
            end_pose_real.append(robot_pose.p)
        end_pose_real = np.array(end_pose_real)

        # #plt.plot(t_traj,curve_bf[:,:3],'--',linewidth = 4)
        # act_t_traj = np.linspace(rt_joint.clock[0],rt_joint.clock[-1], len(curve_bf))
        # plt.plot(act_t_traj,curve_bf[:,:3],'--',linewidth = 4)        
        # plt.plot(rt_joint.clock,end_pose_real)
        # plt.title('tool tip trajectorys: actual(-), planned(---)')
        # plt.xlabel('time (sec)')
        # plt.ylabel('milimeter')
        # plt.legend(['x','y','z'])
        # plt.show()
        
        

    def jog_home(self):
        self.robot.command_mode = self.halt_mode
        time.sleep(0.1)
        self.robot.command_mode = self.position_mode
        ###enable velocity emulation
        self.vel_ctrl.enable_velocity_mode()
        print('velocity control enabled')
        ## home the robot
        self.jog_joint(self.home_q, 2, threshold=0.01)
        time.sleep(2)
        print('robot homed, velocity control disabled')
        self.vel_ctrl.set_velocity_command(np.zeros(6))
        self.vel_ctrl.disable_velocity_mode()
    
    
    def z_touch(self,lift = True):
        R_home = self.home_H.R
        with open('config/ati_pose.yaml') as tool_file:
            H_ati=np.array(yaml.safe_load(tool_file)['H'], dtype=np.float64)
        
        with open('config/default_tcp.yaml') as tool_file:
            H_tcp=np.array(yaml.safe_load(tool_file)['H'], dtype=np.float64)    
        R_ati = H_ati[:3,:3]
        R_tcp = H_tcp[:3,:3]
        d_FT_TCP = 0.0229  # meter, COC -> center of compliance
        F_z_des = 1.7  # N in tool frame
        count = 0
        touchpoint = None
        # ###enable velocity emulation
        # self.vel_ctrl.enable_velocity_mode() 
        while True:
            try:
                #convert ati frame to base frame
                q_cur=self.vel_ctrl.joint_position()
                R=self.robot_toolbox.fwd(q_cur).R
                if np.linalg.norm(self.torque)>1 or np.linalg.norm(self.force)>1:
                    torque_EE=np.dot(R_ati,self.torque)
                    force_EE=np.dot(R_ati,self.force)
                else:
                    torque_EE=np.zeros((3,1))
                    force_EE=np.zeros((3,1))

                force_TCP = R_tcp.T@force_EE
                torque_TCP = R_tcp.T@force_EE
                ####### compliance in z dir
                force_z = force_TCP[-1][0]
                # force_
                torque_y = torque_TCP[1][0]
                F_z_TCP = force_z + (-d_FT_TCP)*torque_y
                ###dynamic damping coefficient
                damping_coeff=max(np.linalg.norm(np.array([force_z, torque_y]))/1.5,1) #/1.5
                b_z = 0.3*damping_coeff  #0.2
                ### v from TCP frame to base frame
                v_z_TCP = (F_z_TCP - F_z_des) / b_z
                # TCP to base frame
                R_bf_EE = R @ R_ati.T  # R_base_end_effector
                R_bf_TCP = R_bf_EE @ R_tcp
                v_bf = R_bf_TCP @ np.array([0,0, v_z_TCP])
                #print('damping', damping_coeff*2000.)
                ###pass to controller

                ###QP controller, cartesian linear motion and orientation
                vd=np.round(np.clip(v_bf, -100, 100),3)  # in mm somehow
                count += 1
                # if count % 100 == 0:
                #     print('vd', vd,
                #           'force_EE:', np.round(force_EE.T[0],3),damping_coeff)
                self.move(vd, np.dot(R, R_home.T))
                #######
                if np.linalg.norm(vd) < 0.06:
                    touchpoint = self.robot_toolbox.fwd(q_cur).p
                    if lift:
                        p_up = R_tcp @ np.array([0,0,10])
                        self.jog_joint_movel(touchpoint+p_up,
                                             10,threshold=0.05, 
                                             acc_range=0., Rd = R_home)
                    break
            except:
                traceback.print_exc()
                break

        return touchpoint
    
    
    def wrench_wire_cb(self,w,value,time):
        self.torque=np.array([value['torque']['x'],value['torque']['y'],value['torque']['z']])
        self.force=np.array([value['force']['x'],value['force']['y'],value['force']['z']])
        

    def move(self, vd, ER):
        '''
        *only use under velociy mode
        
        To solve: min_qdot ||J(q)qdot - [vd; wd]||
        formulating to 1/2 x^T H x + f^T x 
        where x = q_dot

        J' = [J_p; w * J_R] + Kq     note w is for unit matching
        [a; b]^T [a; b] = a^T a + b^T b
        H = J'^T * J' = Jp^T*Jp + w* J_R^T * J_R^T + Kq
        f = -Jp^T * vd - w* JR^T * wd
        '''
        try:
            w=1.
            Kq=.01*np.eye(6)    #small value to make sure positive definite
            KR=np.eye(3)        #gains for position and orientation error

            q_cur=self.vel_ctrl.joint_position()
            J=self.robot_toolbox.jacobian(q_cur)        #calculate current Jacobian
            Jp=J[3:,:]
            JR=J[:3,:] 

            H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)

            H=(H+np.transpose(H))/2  # just to ensure symmetry 


            k,theta = R2rot(ER)
            k=np.array(k)
            s=np.sin(theta/2)*k         #eR2
            wd=-np.dot(KR,s)  
            f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)
            ###Don't put bound here, will affect cartesian motion outcome
            qdot=solve_qp(H, f, solver = 'quadprog')
            ###For safty, make sure robot not moving too fast
            if np.max(np.abs(qdot[:-2]))>1:
                qdot=np.zeros(6)
                print('too fast')
            self.vel_ctrl.set_velocity_command(qdot)

        except:
            traceback.print_exc()
        return

    def jog_joint(self,q,max_v,threshold=0.01,dcc_range=0.1):
        '''
        * only use in velocity mode
        '''
        gain=max(2*max_v,1)
        diff=q-self.vel_ctrl.joint_position()

        while np.linalg.norm(diff)>threshold:

            diff=q-self.vel_ctrl.joint_position()
            diff_norm=np.linalg.norm(diff)
            # qdot=np.where(np.abs(diff) > dcc_range, max_v*diff/np.linalg.norm(diff), gain*diff)
            if diff_norm<dcc_range:
                qdot=gain*diff
            else:
                qdot=max_v*diff/diff_norm

            # if np.max(np.abs(qdot))>0.8:
            # 	qdot=np.zeros(6)
            # 	print('too fast')

            self.vel_ctrl.set_velocity_command(qdot)
            
    def jog_joint_movel(self,p,max_v,threshold=0.001,acc_range=0.01,dcc_range=0.04,Rd=[]):
        '''
        * only use in velocity mode
        '''
        q_cur=self.vel_ctrl.joint_position()
        gain=max(5*max_v,1)
        p_init=self.robot_toolbox.fwd(q_cur).p
        diff=p-p_init
        time_temp=np.linalg.norm(diff)/max_v


        while np.linalg.norm(diff)>threshold:
            q_cur=self.vel_ctrl.joint_position()
            pose_cur=self.robot_toolbox.fwd(q_cur)
            diff=p-pose_cur.p
            diff2=np.linalg.norm(pose_cur.p-p_init)

            diff_norm=np.linalg.norm(diff)
            diff2_norm=np.linalg.norm(diff2)

            v_temp=max_v*diff/diff_norm

            if diff_norm<dcc_range:
                v=gain*diff
            elif diff2_norm<acc_range:
                v=diff2_norm*v_temp/acc_range
            else:
                v=v_temp
            ###correcting orientation
            if len(Rd)==0:
                self.move(v,np.eye(3))
            else:
                self.move(v,np.dot(pose_cur.R,Rd.T))
                
        # self.vel_ctrl.set_velocity_command(np.zeros(6))


if __name__ == "__main__":
    proj = tormach_machining()
    
    curve_bf, t_traj = proj.load_traj()
    #proj.eval_traj(curve_bf, t_traj)
    proj.jog_traj(curve_bf, t_traj, depth = 3, spindle = False)
    
    # data_path = 'data/RPI/'
    # file_list = ['R_outside', 'R_inside',
    #              'P_outside', 'P_inside',
    #              'I_outside']
    # for file in file_list:
    #     traj_gen_curve.main(data_dir=data_path, file_name=file)
    #     curve_bf, t_traj = proj.load_traj(data_path=data_path,data_name=file)
    #     # proj.eval_traj(curve_bf, t_traj)
    #     proj.jog_traj(curve_bf, t_traj, depth = 3)
    proj.jog_home()

    
    