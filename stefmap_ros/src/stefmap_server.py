#!/usr/bin/env python

from stefmap_ros.msg import STeFMapMsg,STeFMapCellMsg
from stefmap_ros.srv import GetSTeFMap
import rospy
import numpy as np
from fremenarray.msg import FremenArrayActionGoal,FremenArrayActionResult, FremenArrayAction, FremenArrayGoal
from os.path import expanduser
import time as t
import actionlib

predicted_probabilities = []
stefmap_pub = None

def handle_GetSTeFMap(req):
	global predicted_probabilities
	global stefmap_pub

	rows = int((req.y_max - req.y_min)/req.cell_size)
	columns = int((req.x_max - req.x_min)/req.cell_size)

	fremenarray_client = actionlib.SimpleActionClient('/fremenarray', FremenArrayAction)
	
	rospy.Duration(3.0)

	if not fremenarray_client.wait_for_server(rospy.Duration(5.0)):
		print "Cannot connect to FremenArray"
		mSTefMap = STeFMapMsg()
		return mSTefMap
	
	frem_msg=FremenArrayGoal()
	frem_msg.operation = 'predict'
	frem_msg.order = req.order
	frem_msg.time = req.header.stamp.to_sec()

	fremenarray_client.send_goal(frem_msg)
	if not fremenarray_client.wait_for_result(rospy.Duration(10.0)):
		print "Error getting the STeF-Map"
		mSTefMap = STeFMapMsg()
		return mSTefMap
	fremenarray_result = fremenarray_client.get_result()


	prob_matrix = np.reshape(fremenarray_result.probabilities,(rows,columns,8))

	# Creating the ros msg to has to be returned to client calling
	mSTefMap = STeFMapMsg()

	mSTefMap.header.frame_id = 'map'
	mSTefMap.header.stamp = req.header.stamp	
	mSTefMap.x_min = req.x_min
	mSTefMap.x_max = req.x_max
	mSTefMap.y_min = req.y_min
	mSTefMap.y_max = req.y_max
	mSTefMap.cell_size = req.cell_size
	mSTefMap.rows = rows
	mSTefMap.columns = columns
	

	# iterate through all the cell and get also the bin with the maximum probability and the associated angle
	index = 0
	for r in range(0,rows):
		for c in range(0,columns):
			stefmap_cell = STeFMapCellMsg()

			stefmap_cell.row = int(r)
			stefmap_cell.column = int(c)
			stefmap_cell.x = float(req.x_min + req.cell_size*0.5 + req.cell_size * c)
			stefmap_cell.y = float(req.y_max - req.cell_size*0.5 - req.cell_size * r)
			stefmap_cell.probabilities = [float(prob_matrix[r,c,0]) ,
										  float(prob_matrix[r,c,1]) ,
										  float(prob_matrix[r,c,2]) ,
										  float(prob_matrix[r,c,3]) ,
										  float(prob_matrix[r,c,4]) ,
										  float(prob_matrix[r,c,5]) ,
										  float(prob_matrix[r,c,6]) ,
										  float(prob_matrix[r,c,7]) ] 

			max_number = -1
			for b in range(0,8):
				if prob_matrix[r,c,b] > max_number:
					max_number = prob_matrix[r,c,b]
					max_orientation = b

			stefmap_cell.best_angle = max_orientation*45

			mSTefMap.cells.append(stefmap_cell)
			index = index + 1

	stefmap_pub.publish(mSTefMap)
	print("STeFMap sent!")
	return mSTefMap


if __name__ == "__main__":
    
	rospy.init_node('get_stefmap_server')
	stefmap_service = rospy.Service('get_stefmap', GetSTeFMap, handle_GetSTeFMap)
	
	stefmap_pub = rospy.Publisher('/stefmap', STeFMapMsg, queue_size=1, latch=True)

	print "Ready to provide STeF-Maps!"
	rospy.spin()
