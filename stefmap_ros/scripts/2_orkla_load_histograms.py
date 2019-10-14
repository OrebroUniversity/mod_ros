import rospy
import sys
import time
import datetime
import math
import os
import numpy as np



if __name__ == '__main__':
	# Map dimensions and cell size
	xmin = -20 # meters
	xmax = 60   # meters
	ymin = -5 # meters
	ymax = 65 # meters
	cell_size = 1 #meters

	# starting and ending time, time interval.
	time_interval = 3600 #seconds
	
	output_file_name = "./../data/orkla_2019_06_13_histogram.txt"
	print("Loading histograms to FreMEn...")
	os.system("python ./tools/load_histograms.py "+output_file_name)
	print("Done")


	output_file_name = "./../data/orkla_2019_06_14_histogram.txt"
	print("Loading histograms to FreMEn...")
	os.system("python ./tools/load_histograms.py "+output_file_name)
	print("Done")
