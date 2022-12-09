# -*- coding: utf-8 -*-
"""
Created on Tue Nov 15 15:32:21 2022

@author: user
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Nov 15 14:56:11 2022

@author: user
"""
#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml, argparse
from importlib import import_module

import sys
sys.path.append('toolbox/')
import Spindle as SC


#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,default='tormach',help="List of camera names separated with commas")
args, _ = parser.parse_known_args()
robot_name=args.robot_name

sys.path.append('../toolbox')
from general_robotics_toolbox import Robot, q2R
#auto discovery
time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.NodeName:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
# 	sys.exit()
	url = '192.168.50.152:139'

print("TORMACH URL = " + str(url))
####################Start Service and robot setup

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

spindle = SC.Controller(robot)

spindle.Start()
if spindle.setSpeed(15000) == True:
 	print("Speed Achieved")
time.sleep(10)
spindle.Stop()

# spindle.setDirection(1)
# time.sleep(2)
# spindle.setDirection(0)

