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
        jog_mode = robot_const["RobotCommandMode"]["jog"]

        print(robot.robot_info.device_info.device.name+" Connected")
        
        if os.path.exists('home_joint_angles.npy'):
            with open('home_joint_angles.npy', 'rb') as f:
                q_top = np.load(f)
            print('Found and loaded home joint config:\n',
                  q_top)
    
        print('*******hands off the robot for initializing jog**********')
        time.sleep(3)

        robot.command_mode = halt_mode
        time.sleep(0.1)
        robot.command_mode = jog_mode
        robot.jog_freespace(q_top,np.ones(6)*0.5,True) ## true is to wait
        print('jog complete, \n',
             'move the tool to [origin, x_lim, y_lim]')
        time.sleep(3)
        robot.command_mode = halt_mode
        time.sleep(0.1)
        robot.command_mode = position_mode
        

    def start(self):
        ###enable velocity emulation
        self.vel_ctrl.enable_velocity_mode()
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

                self.move(K_force*force_bf.T[0], np.eye(3))
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



if __name__ == "__main__":
    manual_tormach = manual_guiding()
    manual_tormach.start()
    time.sleep(10)
    manual_tormach.stop()