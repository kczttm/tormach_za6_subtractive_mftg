from RobotRaconteur.Client import *
import time, sys
import numpy as np
import matplotlib.pyplot as plt
from pandas import *
sys.path.append('toolbox')
from lambda_calc import *
from robot_def import *


time.sleep(5)
#select dataset
data_dir='data/wood/'
#select TCP
with open('config/tcp.yaml') as file:
    H_tcp = np.array(yaml.safe_load(file)['H'],dtype=np.float64)
robot=tormach(R_tool=H_tcp[:3,:3],p_tool=H_tcp[:-1,-1])

###index trajectory with time
vd=100 		#mm/s
curve_js = read_csv(data_dir+'Curve_js.csv',header=None).values
lam=calc_lam_js(curve_js,robot)

lam_diff=np.gradient(lam)
dt=lam_diff/vd
t_traj=np.cumsum(dt)



c = RRN.ConnectService('rr+tcp://[fe80::180d:c0c1:b05b:57f7]:11111/?nodeid=b1141357-a2c8-41dc-b16d-02dc9610ddea&service=tormach_robot')

cmd_w = c.position_command.Connect()
state_w = c.robot_state.Connect()
state_w.WaitInValueValid()

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)
halt_mode = robot_const["RobotCommandMode"]["halt"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",c)

c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = jog_mode

c.jog_freespace(curve_js[0],np.ones(6),False)
print('jog complete')



c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = trajectory_mode



JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",c)
JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",c)
waypoints = []


for i in range(len(curve_js)):

	wp = JointTrajectoryWaypoint()
	wp.joint_position = curve_js[i]
	wp.time_from_start = t_traj[i]
	waypoints.append(wp)

traj = JointTrajectory()
# traj.joint_names = [j.joint_identifier.name for j in c.robot_info.joint_info]
traj.joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
traj.waypoints = waypoints

traj_gen = c.execute_trajectory(traj)

while (True):
	t = time.time()

	try:
		res = traj_gen.Next()
	except RR.StopIterationException:
		print('completed')
		break

plt.plot(time_stamps,joint_positions_history)
plt.plot(time_stamps,desired_position)

plt.title('sin_traj')
plt.show()