import rospy
import sys
import time
import datetime
import math
import os
import numpy as np



if __name__ == '__main__':

	# Map dimensions and cell size
	xmin = -5 # meters
	xmax = 40   # meters
	ymin = -5 # meters
	ymax = 40 # meters
	cell_size = 1 #meters

	# starting and ending time, time interval.
	time_interval = 86400 #seconds
	
#	# BAGFILE 1
#	input_file_name =  "./../data/orkla_2019_10_14_circle.txt"
#	output_file_name = "./../data/orkla_2019_10_14_circle_histogram.txt"
#	t_ini = 1571011200 #14 ocotber 2019
#	t_end = 1571011200 + 86400
#
#	print("Creating flowmap histograms...")
#	os.system("python ./tools/create_flowmap_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
#	print("Done")
#
#	# BAGFILE 2
#	input_file_name =  "./../data/orkla_2019_10_14_updown.txt"
#	output_file_name = "./../data/orkla_2019_10_14_updown_histogram.txt"
#	t_ini = 1571011200 #14 october 2019
#	t_end = 1571011200 + 86400
#
#	print("Creating flowmap histograms...")
#	os.system("python ./tools/create_flowmap_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
#	print("Done")

	input_file_name =  "./../data/orkla_2019_10_14_all.txt"
	output_file_name = "./../data/orkla_2019_10_14_all_histogram.txt"
	t_ini = 1571011200 #14 october 2019
	t_end = 1571011200 + 86400

	print("Creating flowmap histograms...")
	os.system("python ./tools/create_flowmap_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")