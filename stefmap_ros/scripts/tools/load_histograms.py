#!/usr/bin/env python

import rospy
import sys
import time
from std_msgs.msg import String
from fremenarray.msg import FremenArrayActionGoal,FremenArrayActionResult,FremenArrayAction,FremenArrayGoal
import os
import actionlib

if __name__ == '__main__':

	file_name = sys.argv[1]
	

	rospy.init_node('load_data_fremen', anonymous=True)
	fremenarray_client = actionlib.SimpleActionClient('/fremenarray', FremenArrayAction)
	fremenarray_client.wait_for_server()
	
	frem_msg=FremenArrayGoal()
	states = []
	ite = 1
	with open(file_name,"r") as file:
		for line in file:
			current_line = line.split(',')
			timestamp = int(float(current_line[0]))
			for i in range(1,len(current_line)):
				states.append(float(current_line[i]))

			frem_msg.operation = 'add'
			frem_msg.time = timestamp
			frem_msg.states = states

			fremenarray_client.send_goal(frem_msg)
	
			fremenarray_client.wait_for_result()
			fremenarray_result = fremenarray_client.get_result()
			print fremenarray_result.message

			states=[]
			ite = ite +1