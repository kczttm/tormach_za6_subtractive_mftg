from general_robotics_toolbox import *
import numpy as np
import yaml, copy
import pickle

def Rx(theta):
	return np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
def Ry(theta):
	return np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
def Rz(theta):
	return np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
ex=np.array([[1],[0],[0]])
ey=np.array([[0],[1],[0]])
ez=np.array([[0],[0],[1]])

#ALL in mm
class tormach(object):
	#default tool paintgun
	def __init__(self,R_tool=Ry(np.radians(90)),p_tool=np.array([0.45,0,-0.05])*1000.,acc_dict_path=''):
		###ABB IRB 6640 180/2.55 Robot Definition
		self.H=np.concatenate((ez,ey,ey,ex,ey,ex),axis=1)
		p0=np.array([[0],[0],[0.279]])
		p1=np.array([[0.025],[0],[0.171]])
		p2=np.array([[0.],[-0.001],[0.454]])
		p3=np.array([[0.123],[0],[0.035]])   
		p4=np.array([[0.2965],[0.001],[0]])
		p5=np.array([[0.1],[0],[0]])
		p6=np.array([[0.0],[0],[0.0]])

		
		self.R_tool=R_tool
		self.p_tool=p_tool


		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		
		###updated range&vel limit
		self.upper_limit=np.radians([170.,135.,155.,150.,120.,360.])
		self.lower_limit=np.radians([-170.,-100.,-120.,-150.,-120.,-360.])
		self.joint_vel_limit=np.radians([150,112.5,150,204.5,225,360])
		self.joint_acc_limit=np.array([-1,-1,-1,42.49102688076435,36.84030926197994,50.45298947544431])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=p_tool)

		###acceleration table
		if len(acc_dict_path)>0:
			acc_dict= pickle.load(open(acc_dict_path,'rb'))
			q2_config=[]
			q3_config=[]
			q1_acc=[]
			q2_acc=[]
			q3_acc=[]
			for key, value in acc_dict.items():
			   q2_config.append(key[0])
			   q3_config.append(key[1])
			   q1_acc.append(value[0])
			   q2_acc.append(value[1])
			   q3_acc.append(value[2])
			self.q2q3_config=np.array([q2_config,q3_config]).T
			self.q1q2q3_acc=np.array([q1_acc,q2_acc,q3_acc]).T

	def get_acc(self,q_all):
		#if a single point
		if q_all.ndim==1:
			###find closest q2q3 config, along with constant last 3 joints acc
			idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[1:3],axis=1))
			return np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:])
		else:
			acc_limit_all=[]
			for q in q_all:
				idx=np.argmin(np.linalg.norm(self.q2q3_config-q[1:3],axis=1))
				acc_limit_all.append(np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:]))

		return np.array(acc_limit_all)


	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0]),qlim_override=False):
		if qlim_override:
			robot_def=copy.deepcopy(self.robot_def)
			robot_def.joint_upper_limit=999*np.ones(len(self.upper_limit))
			robot_def.joint_lower_limit=-999*np.ones(len(self.lower_limit))
			pose_temp=fwdkin(robot_def,q)
		else:
			pose_temp=fwdkin(self.robot_def,q)

		pose_temp.p=base_R@pose_temp.p+base_p
		pose_temp.R=base_R@pose_temp.R
		return pose_temp

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=self.fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		return q_all

class Transform_all(object):
	def __init__(self, p_all, R_all):
		self.R_all=np.array(R_all)
		self.p_all=np.array(p_all)


def main():
	angle=np.radians(25)
	R_tool=Rx(angle)@Ry(np.pi)
	print(R_tool)
	p_tool=[25,100*np.sin(angle),-100*np.cos(angle)]
	print(p_tool)
	
	H=np.eye(4)
	H[:-1,-1]=p_tool
	H[:3,:3]=R_tool

	with open(r'../config/tcp.yaml', 'w') as file:
		documents = yaml.dump({'H':H.tolist()}, file)


	return

if __name__ == '__main__':
	main()