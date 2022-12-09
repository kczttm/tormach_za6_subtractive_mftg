# -*- coding: utf-8 -*-
"""
Created on Tue Nov 15 14:56:11 2022

Author: Trenton Squires
Email: trenton.c.squires@gmail.com
"""

from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml, argparse
from importlib import import_module


class Controller(object):
	def __init__(self,robot):
		# =====================================================================
		# Tormach (TM) IOs are one higher on Pathpilot software and code
		# compared to back of physical Tormach control box.
		# Physical IO 8 cooresponds to Virtual IO 9
		# =====================================================================
		
		self.robot = robot
		self.io09 = '9'   #TM IO to Spindle I - Pin 14, O - Pin 06
		self.io10 = '10'  #TM IO to Spindle I - Pin 02, O - Pin 07
		self.io11 = '11'  #TM IO to Spindle I - Pin 03, O - Pin 21
		self.io12 = '12'  #TM IO to Spindle I - Pin 04, O - Pin 08
		self.io13 = '13'  #TM IO to Spindle I - Pin 15, O - Pin 19
		self.timeout = 3 # seconds
		self.pulsetime = 0.05 # seconds - 50% Duty Cycle Pulse
		self.speed = 0 # Initialize speed at 0
		
	def Start(self):
		# =====================================================================
		# High - Start
		# NOTE: MUST ENSURE SPINDLE STARTS AT 1000 RPM ON CONTROLLER
		# =====================================================================
		
		self.robot.setf_signal(self.io09,1)
		self.speed = 1000
		return True
	
	def Stop(self):
		# =====================================================================
		# Low - Stop
		# Resets the speed to 1000 rpm
		# =====================================================================
		self.setSpeed(1000)
		self.robot.setf_signal(self.io09,0)
		self.speed = 0
		return True
	
	def isRotating(self):
		# =====================================================================
		# High - Yes
		# Low - No
		# =====================================================================

		if self.robot.getf_signal(self.io09) == 1:
			return True
		else:
			return False
		
	def setDirection(self, direction):
		# =====================================================================
		# High - Reverse ("Counter-Clockwise")
		# Low - Forward ("Clockwise")
		# =====================================================================
		if direction == 0:
			self.robot.setf_signal(self.io10,0)
			print("Forward")
		elif direction == 1:
			self.robot.setf_signal(self.io10,1)
			print("Reverse")
		else:
			print('Direction Arugment must be CW (Forward) or CCW (Reverse) (clockwise or counter-clockwise)')
		return True
	def getDirection(self):
		# =====================================================================
		# High - Reverse
		# Low - Forward
		# =====================================================================

		if self.robot.getf_signal(self.io10) == 0:
			return "Forward"
		else:
			return "Reverse"
	
	def setSpeed(self, speedSetpoint):
		# =====================================================================
		# Count Pulse For Motor Speed
		# High - Closed
		# Low - Open
		# For every pulse, a 1000 min^-1 increment is applied (Speed up or down 
		# based on status of Pin 15 (UD-IN)
		# Based on assumption that the Controller is initialized at 1000 rpm
		# =====================================================================

		roundedSpeed = round(speedSetpoint,-3) # Can only set speed to nearest 1000
		sign = 0
		if int(roundedSpeed) <= 80000 and int(roundedSpeed) >= 0:
			speedDifference = abs(roundedSpeed - self.speed)
			print("speedDifference = " +str(speedDifference))
			if roundedSpeed > self.speed:
				self.robot.setf_signal(self.io13,1) # Set to speed up
				sign = 1
			elif roundedSpeed < self.speed:
				self.robot.setf_signal(self.io13,0) # Set to speed down
				sign = -1
			elif roundedSpeed == self.speed:
				print("Spindle speed already at Setpoint")
				return False
			for i in range(int(speedDifference/1000)):
				self.robot.setf_signal(self.io11,1)
				time.sleep(self.pulsetime)
				self.robot.setf_signal(self.io11,0)
				time.sleep(self.pulsetime)
				self.speed = self.speed+sign*1000
			return True
		else:
			print("Speed Setpoint must be between 0 and 80000")
			return False
	
	def isSpeedAcheived(self):
		# =====================================================================
		# High - Speed is achieved (> 90% of set speed)
		# Low - Speed is Not achieved (<90% of set speed)
		# =====================================================================

		if self.robot.getf_signal(self.io11) == 1:
			return True
		else:
			return False
	
	def clearError(self):
		# =====================================================================
		# High - Closed
		# Low - Open
		# Pulse (off - on - off) to reset error
		# =====================================================================
		
		self.robot.setf_signal(self.io12,0)
		time.sleep(self.pulsetime)
		self.robot.setf_signal(self.io12,1)
		time.sleep(self.pulsetime)
		self.robot.setf_signal(self.io12,0)
		return True
	
	def isError(self):
		# =====================================================================
		# High - Normal operation
		# Low - Error
		# =====================================================================

		if self.robot.getf_signal(self.io12) == 0:
			return True
		else:
			return False