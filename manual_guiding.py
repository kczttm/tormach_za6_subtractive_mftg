from RobotRaconteur.Client import *
import numpy as np
import time, sys, yaml, os, threading, traceback
from importlib import import_module
from general_robotics_toolbox import *

sys.path.append('toolbox')
from vel_emulate_sub import EmulatedVelocityControl
from qpsolvers import solve_qp
from robots_def import *

class manual_guiding():
    def __init__(self):
        
        self.stop_event = threading.Event()
        self._lock = threading.RLock()
        
        self.torque=np.zeros(3)
        self.force=np.zeros(3)

        with open('config/tormach_za06_robot_default_config.yml') as robot_file:
            with open('config/tool_pose_modified.yaml') as tool_file:
                self.robot_toolbox=yml2robdef(robot_file,tool_file)
        ################################Connect to FT Sensor###################################
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
        robot=robot_sub.GetDefaultClientWait(1)
        state_w = robot_sub.SubscribeWire("robot_state")
        cmd_w = robot_sub.SubscribeWire("position_command")
        RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
        self.vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)

        robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
        state_flags_enum = robot_const['RobotStateFlags']
        halt_mode = robot_const["RobotCommandMode"]["halt"]
        position_mode = robot_const["RobotCommandMode"]["position_command"]
        #jog_mode = robot_const["RobotCommandMode"]["jog"]

        print(robot.robot_info.device_info.device.name+" Connected")
        
        if os.path.exists('home_joint_angles.npy'):
            with open('home_joint_angles.npy', 'rb') as f:
                q_top = np.load(f)
            print('Found and loaded home joint config:\n',
                  q_top)
        
        self.home_H = self.robot_toolbox.fwd(q_top)
        self.home_q = q_top
        print('*******hands off the robot for initializing jog**********')
        robot.command_mode = halt_mode
        time.sleep(0.1)
        robot.command_mode = position_mode
        

    def start(self, homing = True):
        ###enable velocity emulation
        self.vel_ctrl.enable_velocity_mode()
        if homing:
            ## home the robot
            self.jog_joint(self.home_q, 2, threshold=0.1)
            time.sleep(2)
            print('robot homed, thread starting')
        self._checker = threading.Thread(target = self.joint_loose,
                                         args = (self.stop_event,))
        self._checker.daemon = True
        self._checker.start()
        

    def joint_loose(self, event):
        while not self.stop_event.is_set():
            while self._lock:
                #convert tool frame to base frame
                q_cur=self.vel_ctrl.joint_position()
                R=self.robot_toolbox.fwd(q_cur).R
                if np.linalg.norm(self.torque)>1 or np.linalg.norm(self.force)>1:
                    torque_bf=np.dot(R,self.torque)
                    force_bf=np.dot(R,self.force)
                else:
                    torque_bf=np.zeros((3,1))
                    force_bf=np.zeros((3,1))
                #command motion based on reading
                K_force=10 #10
                K_torque=10 #10
                
                # with correcting orientation
                Rd = self.home_H.R
                self.move(K_force*force_bf.T[0], np.dot(R,Rd.T))
                print(np.round(self.robot_toolbox.fwd(q_cur).p,3))
                
                if event.is_set():
                    self.vel_ctrl.set_velocity_command(np.zeros(6))
                    self.vel_ctrl.disable_velocity_mode()
                    print('velocity control disabled')
                    return
    
    
    def stop(self):
        self.stop_event.set()
        self._checker.join()
    
    
    def wrench_wire_cb(self,w,value,time):
        self.torque=np.array([value['torque']['x'],value['torque']['y'],value['torque']['z']])
        self.force=np.array([value['force']['x'],value['force']['y'],value['force']['z']])
        

    def move(self, vd, ER):
        '''
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


if __name__ == "__main__":
    manual_tormach = manual_guiding()
    manual_tormach.start(homing = True)
    time.sleep(20)
    manual_tormach.stop()