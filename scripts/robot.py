#!/usr/bin/env python
from __future__ import division
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
from std_msgs.msg import Bool


class Robot():

	def __init__(self):

		rospy.init_node('Robot') 
		self.config = read_config()
		self.poseArray = None

		rospy.Subscriber("/map", OccupancyGrid, self.handleMapMessage)

		rospy.Subscriber("/base_scan", LaserScan, self.handleLaserMessage)
		
		self.particlePublisher = rospy.Publisher("/particlecloud", PoseArray, queue_size=10, latch=True)

		self.lMapPublisher = rospy.Publisher("/likelihood_field", OccupancyGrid, queue_size=10, latch=True)
		self.resultUpdatePub = rospy.Publisher("/result_update", Bool, queue_size=10, latch=True)
		
		self.simComplPub = rospy.Publisher("/sim_complete", Bool, queue_size=10)

		self.p_i = 0	
		self.particleArray = []
		self.angleMin = 0
		self.poseArray = PoseArray() 	
		self.poseArray.header.stamp = rospy.Time.now()
		self.poseArray.header.frame_id = 'map'
		self.poseArray.poses = []
		self.increment = 0	
		self.index = 0
		self.first = 0	
		self.laserVals = None	
		self.map = None
		self.newWeights = []	
		self.mAngle = None
		self.mDist = None 
		self.mSteps = None
		self.PzForAllParticles = []
		self.totalPzForAllParticles = []
		
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
		probObsGivenLaser = eExp
		self.lMap.grid = probObsGivenLaser.reshape(self.lMap.grid.shape) 

		self.occupancyGridMsg = self.lMap.to_message()		
		
		self.lMapPublisher.publish(self.occupancyGridMsg) 
		
	def handleLaserMessage(self, message):
		self.laserVals = message.ranges 
		self.angleMin = message.angle_min #what are these in radians?
		self.increment = message.angle_increment		
			
	def calculatePzGivenLaserScan(self):
		PzForParticleI = []	
		p = self.p_i
		for l in range ( len (self.laserVals) ):	
			self.rLaserAngle = self.angleMin + l*self.increment
			totalLaserAngle = self.particleArray[p][2] + self.rLaserAngle

			corX = self.particleArray[p][0] + self.laserVals[l] * m.cos(totalLaserAngle)				
			corY = self.particleArray[p][1] + self.laserVals[l] * m.sin(totalLaserAngle)

			[x, y] = self.lMap.cell_position(corX, corY)
			self.LP = self.lMap.get_cell(x, y)
			if self.LP == self.LP:
				self.P_z = self.LP*self.config["laser_z_hit"] + self.config["laser_z_rand"]
				PzForParticleI.append(deepcopy(self.P_z))
		self.PzForAllParticles.append(deepcopy(PzForParticleI))#is this creating a deep copy?

	def weighAndResampleParticles(self):
		#----------------------------------------------------------------------
		#there are some edge cases: please take care of all edge cases
		#---------------------------------------------------------------------
		self.newWeights = []
		self.PzForAllParticles = []
		for p in range ( len (self.particleArray)):
			self.p_i = p	
			self.calculatePzGivenLaserScan()	

		self.totalPzForAllParticles = []
		self.calculateTotalPz()		

		with open ("lengthlog.txt", 'a') as infile:
			infile.write("self.totalPzForAllParticles")
			infile.write(str(len(self.totalPzForAllParticles)))
			infile.write("\n")

		self.calculateNewWeight()
		self.normalizeWeights()
		self.resampleParticles()

	def calculateTotalPz(self):
		for i in range(len(self.PzForAllParticles)):
			totalPzForParticleI = 0.0 
			for j in range(len(self.PzForAllParticles[i])):		
				Pz_i = self.PzForAllParticles[i][j]
				totalPzForParticleI += Pz_i * Pz_i * Pz_i 

			self.totalPzForAllParticles.append(deepcopy(totalPzForParticleI))
			
	def calculateNewWeight(self):
		for o in range ( len (self.totalPzForAllParticles) ):
			#calculate the new weight from the old weights
			newW = 0.0
			newW = self.particleArray[o][3]*self.totalPzForAllParticles[o]
			self.particleArray[o][3] = deepcopy( newW )
			with open ("lengthlog.txt", 'a') as infile:
				infile.write("particleArray[o][3]:")
				infile.write(str(self.particleArray[o][3]))
				infile.write("\n")
		
		for k in range (len(self.particleArray) ):
	#particle goes out of the bounds because of the move step update right?	
			x = self.particleArray[k][0]
			y = self.particleArray[k][1]
			temp = (self.lMap.get_cell(x, y))
			if temp != temp: 
				self.particleArray[k][3] = 0
	

	def normalizeWeights(self):		

		normalizeWeight = 0.0	
		for k in range (len (self.particleArray) ):	
			normalizeWeight += self.particleArray[k][3]
		
		for k in range (len (self.particleArray) ):
			newNormalW = 0.0
			newNormalW = self.particleArray[k][3]/normalizeWeight
			self.particleArray[k][3] = deepcopy(newNormalW)
			self.newWeights.append(deepcopy( self.particleArray[k][3]) )

		#printing the normalized sum
		summ = 0.0	
		for n in range( len (self.newWeights) ):
			summ += self.newWeights[n]
		with open ("normalog.txt", 'a') as infile:
			infile.write(str(summ))
			infile.write("\n")
		

	def resampleParticles(self):	
		resampleArray = []
		for r in range ( 800 ):
			particleAdd = []
			resampleP = np.random.choice(800, 1, replace=True, p=self.newWeights)
			particleAdd = self.particleArray[ resampleP[0] ]	
			resampleArray.append( deepcopy(particleAdd) )	
			if not resampleArray[r][3]:
				with open ("reweighlog.txt", 'a') as infile:
					infile.write("weight of 0 particle added ")
					infile.write("\n")
						
			resampleArray[r][0] += random.gauss(0, self.config["resample_sigma_x"])
			resampleArray[r][1] += random.gauss(0, self.config["resample_sigma_y"])
			resampleArray[r][2] += random.gauss(0, self.config["resample_sigma_angle"])

		with open ("resample.txt", 'a') as infile:
			infile.write("resample array")
			infile.write( str(len (resampleArray) ) )
			infile.write( str( (resampleArray) ) )
			infile.write("\n")

		self.particleArray = deepcopy(resampleArray)

		#with open ("reweighlog.txt", 'a') as infile:
			#infile.write("len (self.totalPzForAllParticles)")
			#infile.write( str(len (self.totalPzForAllParticles)) )
			#infile.write("\n")
		#next using the above totalPzfor each particle I must calculate the new weight
		#for each particles and update the Particle array

	def moveParticles(self):	
		self.moveList = self.config["move_list"]	
		for i in range (len (self.moveList)):
			self.first = i	
			self.mAngle = self.moveList[i][0]
			self.mDist = self.moveList[i][1]
			self.mSteps = self.moveList[i][2]

			with open ("moveparticleslog.txt", 'a') as infile:
				infile.write("mDist, mAngle, mSteps")
				infile.write(str(self.mDist))
				infile.write(str(self.mAngle))
				infile.write(str(self.mSteps))
				infile.write("\n")
			#move robot by the angle 
			hf.move_function( m.radians( self.mAngle ), float(0.0) ) 
			#turn by angle, the particles and add noise only for the first move	
			for p in range(len (self.particleArray)): 	
				self.particleArray[p][2] += m.radians(self.mAngle)	
				if i == 0:	
					self.particleArray[p][2] += random.gauss(0, self.config["first_move_sigma_angle"])
			#move particles by m.Dist mStep times
			for j in range (self.mSteps):	
				hf.move_function(0.0, self.mDist)
				#if self.first == 0:
				self.stepUpdate()

			with open ("moveparticleslog.txt", 'a') as infile:
				infile.write("before entering the weigh particles fucntion")
				infile.write("\n")	
			#if self.first == 0:

			#weigh and resample particles here	
			self.weighAndResampleParticles()	
			
			self.poseArray.poses = []		
			for p in range(len (self.particleArray)): 	
				self.pose = hf.get_pose(self.particleArray[p][0], self.particleArray[p][1], self.particleArray[p][2])
				self.poseArray.poses.append(self.pose)
				self.particleArray[p][4] = self.pose
			#if self.first == 0:	
			#publishing particles to particle cloud
			self.particlePublisher.publish(self.poseArray)
			#self.timer = rospy.Timer(rospy.Duration(0.1), self.publishMoveParticles)
        	
	def stepUpdate(self):

		with open ("moveparticleslog.txt", 'a') as infile:
			infile.write("inside stepUpdate: making move: self.mDist")
			infile.write(" ")
			infile.write(str(self.mDist))
			infile.write("\n")

		for k in range (len (self.particleArray) ):		
			self.particleArray[k][0] = self.particleArray[k][0] + self.mDist*m.cos(self.particleArray[k][2])
			self.particleArray[k][1] = self.particleArray[k][1] + self.mDist*m.sin(self.particleArray[k][2]) 
			#why do we add noise only for the 1st move?	
			
			if self.first  == 0:
				self.particleArray[k][0] += random.gauss(0, self.config["first_move_sigma_x"])
				self.particleArray[k][1] += random.gauss(0, self.config["first_move_sigma_y"])
				self.particleArray[k][2] += random.gauss(0, self.config["first_move_sigma_angle"])

	def publishMoveParticles(self):
		with open ("publishMP.txt", 'a') as infile:
			infile.write("inside the publish Move particles fucntion")
		self.particlePublisher.publish(self.poseArray)


if __name__ == '__main__':

	with open ("log.txt", 'a') as infile:
		infile.write("please write")
	robot = Robot() 
