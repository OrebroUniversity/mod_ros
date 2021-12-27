#!/usr/bin/python3

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from fremenarray.msg import FremenArrayActionGoal, FremenArrayGoal, FremenArrayAction
from geometry_msgs.msg import PoseArray, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
import actionlib
import math
import numpy as np
import tf
from stefmap_ros.msg import STeFMapMsg,STeFMapCellMsg
from stefmap_ros.srv import GetSTeFMap, UpdateSTeFMap

class STeFmap_node_offline(object):

	def __init__(self):
		#parameters
		self.grid_size = rospy.get_param('~grid_size',1)#meters		
		self.x_min = rospy.get_param('~x_min',-50)#meters
		self.x_max = rospy.get_param('~x_max',50) #meters
		self.y_min = rospy.get_param('~y_min',-50)#meters
		self.y_max = rospy.get_param('~y_max',50) #meters
		self.num_bins = rospy.get_param('~num_bins',8) #bins dividing the circumference
		self.frame_id = rospy.get_param('~frame_id',"/map")

		# initialize visibility map
		self.width = int((self.x_max-self.x_min)/self.grid_size) ## [cells]
		self.height = int((self.y_max-self.y_min)/self.grid_size) ## [cells]

		# connect to fremenarray
		rospy.loginfo("waiting for FremenArray.....")
		self.fremenarray_client = actionlib.SimpleActionClient('/fremenarray', FremenArrayAction)
		self.fremenarray_client.wait_for_server()
		rospy.loginfo("FremenArray ready!")

		# initiliatize stefmap
		rospy.loginfo("Initializing STeF-map.....")
		self.bin_counts_matrix = np.ones([self.width,self.height,self.num_bins])*-1 #initialize cells as unexplored/unknow
		self.bin_counts_matrix_accumulated = np.zeros([self.width,self.height,self.num_bins])
		self.entropy_map_store = []

		# create stef map service and topic
		get_stefmap_service = rospy.Service('get_stefmap', GetSTeFMap, self.handle_GetSTeFMap)
		self.stefmap_pub = rospy.Publisher('/stefmap', STeFMapMsg, queue_size=1, latch=True)

		self.run()

	def handle_GetSTeFMap(self,req):
		mSTefMap = STeFMapMsg()
		mSTefMap.header.stamp = rospy.get_rostime() 
		mSTefMap.header.frame_id = self.frame_id
		mSTefMap.prediction_time = req.prediction_time
		mSTefMap.x_min = self.x_min
		mSTefMap.x_max = self.x_max
		mSTefMap.y_min = self.y_min
		mSTefMap.y_max = self.y_max
		mSTefMap.cell_size = self.grid_size
		mSTefMap.rows = self.width
		mSTefMap.columns = self.height

		print("STeFMap sent!")
		return mSTefMap

	def run(self):				
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('stefmap_node', anonymous=True)
	stef = STeFmap_node_offline()
