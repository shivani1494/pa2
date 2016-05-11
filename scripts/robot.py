#!/usr/bin/env python

import logging
from scipy import spatial
from sklearn.neighbors import KDTree
import rospy
import json
from math import *
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Quaternion, Point, Twist
from sensor_msgs.msg import LaserScan
import tf
import random
from random import gauss
import math as m
import numpy as np
from copy import deepcopy
from read_config import read_config
from math import exp
import map_utils as mu 
import helper_functions as hf

class Robot():

	def __init__(self):

		rospy.init_node('Robot') 

		#logging.basicConfig(filename='errorPrint.log',level=logging.DEBUG)
		#a = 1
		#logging.debug('This message should go to the log file %d', a)

		self.config = read_config()
		self.poseArray = None
		rospy.Subscriber("/map", OccupancyGrid, self.handleMapMessage)

		self.particlePublisher = rospy.Publisher("/particlecloud", PoseArray, queue_size=10, latch=True)

		rospy.Subscriber("/base_scan", LaserScan, self.handleLaserMessage)

		self.lMapPublisher = rospy.Publisher("/likelihood_field", OccupancyGrid, queue_size=10, latch=True)
		self.particleArray = []
		self.angleMin = 0
		self.poseArray = PoseArray() 	
		self.increment = 0	
		self.index = 0	
		self.laserVals = None	
		self.map = None
		while self.map == None:
			rospy.sleep(0.1)
			
		self.initializeParticles()
		rospy.sleep(0.1)	

		self.constructLMap()
		rospy.sleep(0.1) #why the heck do I need these?!
		
		with open ("log1.txt", 'a') as infile:
			infile.write("from init robot function")
	
		while self.laserVals == None:
			rospy.sleep(0.1)	

		rospy.sleep(0.1)	
		
		self.moveParticles()

		rospy.sleep(0.1)	

		while not rospy.is_shutdown():
			rospy.sleep(0.5)

	def handleMapMessage(self, message):
	 	if self.index == 0:	
			self.map = mu.Map(message)

			print self.map.grid 
			print self.map.cell_position(0, 0) 

			self.lMap = deepcopy(self.map) #how to make a deep copy????
			#self.lMapPublisher.publish(self.lMap.to_message())
			self.index = self.index + 1

	def initializeParticles(self):	
		#initialize all my particles
		#self.particleArray = []
		
		self.poseArray.header.stamp = rospy.Time.now()
		self.poseArray.header.frame_id = 'map'
		self.poseArray.poses = []
		self.particle = []
		self.numP = self.config["num_particles"]
		for i in range(self.numP ):
			#self.particle is an array 
				
			self.particle = np.random.uniform(0, self.map.width, 1)
			self.particle = np.append(self.particle, np.random.uniform(0, self.map.height, 1) )
			self.particle = np.append(self.particle, np.random.uniform(0, 6.28, 1) )
		
			self.particle = np.append(self.particle, np.array([1.0/800.0] ) )

			self.pose = hf.get_pose(self.particle[0], self.particle[1], self.particle[2])
			self.poseArray.poses.append(self.pose)
			self.particle = np.append(self.particle, np.array(self.pose))

			self.particleArray.append(self.particle)
		
		print ("particle publishing")
		self.particlePublisher.publish(self.poseArray)
	
	def constructLMap(self):
		self.obstacleArray = []
		self.allPositions = []	
		#build your obstacle array 
		for i in range( len(self.map.grid) ):	
			for j in range( len(self.map.grid[0])):	
				[x, y] = self.map.cell_position(i, j) 
				if self.map.get_cell(x,y) == 1.0:
					self.obstacleArray.append(np.array(self.map.cell_position(i, j))) 
					#print self.map.cell_position(i, j)	
				self.allPositions.append(np.array(self.map.cell_position(i, j)))  
		#pass it into kdtree
		eExp = []
	
		kdt = KDTree(self.obstacleArray)
		dists = kdt.query(self.allPositions, k=1)[0][:]	
		self.laserStdDev = self.config["laser_sigma_hit"]
		constant = 1.0/( m.sqrt( 2 * m.pi) * self.laserStdDev )
		eExp = np.exp(-0.5*( dists**2  )/( self.laserStdDev**2 ) )
		probObsGivenLaser = constant * eExp
		self.lMap.grid = probObsGivenLaser.reshape(self.lMap.grid.shape) 

		self.occupancyGridMsg = self.lMap.to_message()		
		
		self.lMapPublisher.publish(self.occupancyGridMsg) 
		
	def handleLaserMessage(self, message):
		self.laserVals = message.ranges 
		self.angleMin = message.angle_min #what are these in radians?
		self.increment = message.angle_increment		

	#place all variables into the init fucntion
	def weighParticles(self):
		#----------------------------------------------------------------------
		#there are some edge cases: please take care of all edge cases I beg you!
		#---------------------------------------------------------------------
		#this is the a 2D array, every row corresponds to every particle
		#every column corresponds to Pz values for every laser scan
		#so if we have 100 laser scan values, we have 100 Pz, one for each location
		#laser scan P_z values-->horizontal-->PzArrayForParticleI array used below
		#|___|____|____|____|___||#this for Particle i
		#|___|____|____|____|___||all particles in the vertical
		#|___|____|____|____|___||
		self.PzForAllParticles = []
		for p in range ( len (self.particleArray)):
			PzForParticleI = []	
			for l in range ( len (self.laserVals) ):	
				self.rLaserAngle = self.angle_min + l*self.increment
				totalLaserAngle = self.particleArray[p][2] + self.rLaserAngle
				corX = particleArray[p][0] + self.laserVals[l]*m.cos(totalLaserAngle) 		
				corY = particleArray[p][1] + self.laserVals[l]*m.sin(totalLaserAngle) 	

				#this returns me the mid cell position
				[x, y] = self.lMap.cell_position(corX, corY)
				self.LP = self.lMap.get_cell(x, y)
				#this gets me the P of that cell being an obstacle
				#given my current pos what is the P that I hit the obstacle
				if self.LP != float('nan'):
					self.P_z = self.LP*self.config["laser_z_hit"] + self.config["laser_z_rand"]
					PzForParticleI.append(self.P_z)

			self.PzForAllParticles.append(PzForParticleI)#is this creating a deep copy?
		

		#we are calculating a good P_Z values by somehow summing up all the weights
		#we got in Array PzForParticleI above
		#this we must do over all particles	
		#so here we use the cubes formula but this may change
		self.totalPzForAllParticles = []
		for i in range(len(PzForAllParticles)):
			totalPzForParticleI = 1 
			for j in range(len(self.PzForAllParticles[i])):		
				totalPzForParticleI +=  probArray[i]*probArray[i]*probArray[i]
				totalPzForAllParticles.append(totalPzForParticleI)

		#next using the above totalPzfor each particle I must calculate the new weight
		#for each particles and update the Particle array
		for m in range ( len (self.totalPzForAllParticles) ):
			#calculate the new weight from the old weights
			self.particleArray[m][3] *= totalPzForAllParticles[m]
		
	
	def moveParticles(self):	
		self.moveList = self.config["move_list"]		
		for i in range (len (self.moveList)):
				
			self.mAngle = self.moveList[i][0]
			self.mDist = self.moveList[i][1]
			self.mSteps = self.moveList[i][2]

			with open ("moveparticleslog.txt", 'a') as infile:
				infile.write("making move: self.mAngle, self.mDist")
				infile.write(str(self.mAngle))
				infile.write("\n")
			#move robot 
			hf.move_function( m.radians( self.mAngle ), float(0.0)) 
			#move the particles and add noise only for the first move	
			for p in range(len (self.particleArray)): 	
				self.particleArray[p][2] += m.radians(self.mAngle)	
				if i == 0:	
					self.particleArray[p][2] += random.gauss(0, self.config["first_move_sigma_angle"])
			#move particles
			for j in range (self.mSteps):	
				hf.move_function(0, self.mDist)

				with open ("moveparticleslog.txt", 'a') as infile:
					infile.write("making move: self.mDist")
					infile.write(" ")
					infile.write(str(self.mDist))
					infile.write("\n")

				for k in range (len (self.particleArray) ):		
					self.particleArray[k][0] = self.particleArray[k][0] + self.mDist*m.cos(self.particleArray[k][2])
					self.particleArray[k][1] = self.particleArray[k][1] + self.mDist*m.sin(self.particleArray[k][2]) 
					#why do we add noise only for the 1st move?	
					
					if i == 0:
						self.particleArray[k][0] += random.gauss(0, self.config["first_move_sigma_x"])
						self.particleArray[k][1] += random.gauss(0, self.config["first_move_sigma_y"])
						self.particleArray[k][2] += random.gauss(0, self.config["first_move_sigma_angle"])
			
			self.poseArray.header.stamp = rospy.Time.now()
			self.poseArray.header.frame_id = 'map'
			self.poseArray.poses = []
			rospy.loginfo("logging errorsssso")	
			for p in range(len (self.particleArray)): 	
				self.pose = hf.get_pose(self.particleArray[p][0], self.particleArray[p][1], self.particleArray[p][2])
				self.poseArray.poses.append(self.pose)
				self.particleArray[p][4] = self.pose
			
			self.particlePublisher.publish(self.poseArray)
			with open ("publishMP.txt", 'a') as infile:
				infile.write("outside publishMP")
			self.timer = rospy.Timer(rospy.Duration(0.1), self.publishMoveParticles)
        	

	def publishMoveParticles(self):
		with open ("publishMP.txt", 'a') as infile:
			infile.write("please write")
		self.particlePublisher.publish(self.poseArray)


if __name__ == '__main__':

	with open ("log.txt", 'a') as infile:
		infile.write("please write")
	robot = Robot() 
