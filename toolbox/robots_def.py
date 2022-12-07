from general_robotics_toolbox import *
import numpy as np
import yaml


def Rx(theta):
	return np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
def Ry(theta):
	return np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
def Rz(theta):
	return np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
ex=np.array([[1],[0],[0]])
ey=np.array([[0],[1],[0]])
ez=np.array([[0],[0],[1]])



class arb_robot(object):
	#R_tool make tool z pointing to +x at 0 config
	def __init__(self, H,P,joint_type,upper_limit,lower_limit, joint_vel_limit,H_tool=np.eye(4)):
		###All in mm
		self.H=H
		self.P=P

		###updated range&vel limit
		self.joint_type=joint_type
		self.upper_limit=upper_limit
		self.lower_limit=lower_limit
		self.joint_vel_limit=joint_vel_limit
		self.joint_acc_limit=10*self.joint_vel_limit
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=H_tool[:3,:3],p_tool=H_tool[:-1,-1])

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_temp=fwdkin(self.robot_def,q)
		pose_temp.p=np.dot(base_R,pose_temp.p)+base_p
		pose_temp.R=np.dot(base_R,pose_temp.R)
		return pose_temp

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3)):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose)
		return q_all

def yml2robdef(robot_file,tool_file):
	robot_yml=yaml.full_load(robot_file)
	kin_chain=robot_yml['chains'][0]
	joint_info=robot_yml['joint_info']

	tool_yml=yaml.full_load(tool_file)
	H_tool=np.array(tool_yml['H'])

	###kin chain
	H = []
	P = []

	for i in range(len(kin_chain['H'])):
		H.append(list(kin_chain['H'][i].values()))
		P.append(list(kin_chain['P'][i].values()))
	P.append(list(kin_chain['P'][-1].values()))
	H=np.array(H).reshape((len(kin_chain['H']),3)).T
	P=np.array(P).reshape((len(kin_chain['P']),3)).T*1000	###make sure in mm

	###joint info
	joint_type=[]	
	upper_limit=[]
	lower_limit=[]
	joint_vel_limit=[]
	for i in range(len(joint_info)):
		joint_type.append(0 if joint_info[i]['joint_type']=='revolute' else 1)
		upper_limit.append(joint_info[i]['joint_limits']['upper'])
		lower_limit.append(joint_info[i]['joint_limits']['lower'])
		joint_vel_limit.append(joint_info[i]['joint_limits']['velocity'])

	###create a robot
	robot=arb_robot(H,P,joint_type,upper_limit,lower_limit, joint_vel_limit,H_tool)

	return robot
