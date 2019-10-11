#!/usr/bin/env python

import sys
import rospy
from stefmap_ros.srv import GetSTeFMap
from std_msgs.msg import Header


if __name__ == "__main__":
    prediction_time = 0
    x_min = -10 # meters
    x_max = 60   # meters
    y_min = -10 # meters
    y_max = 50 # meters
    cell_size = 1
    order = 1

    print "Requesting STeF-Map with the following parameters:\n"
    print "  Time to predict: %s"%(prediction_time)
    print "  Fremen Order: %s"%(order)
    print "  X min: %s"%(x_min)
    print "  X max: %s"%(x_max)
    print "  Y min: %s"%(y_min)
    print "  Y max: %s"%(y_max)
    print "  Cell size:%s"%(cell_size)

    rospy.wait_for_service('get_stefmap')
    
    while 1==1:
        try:
            prediction_time = prediction_time + 1800
            get_stefmap = rospy.ServiceProxy('get_stefmap', GetSTeFMap)
            h = Header()
            h.stamp.secs = prediction_time
            h.frame_id = 'map2'
            stefmap = get_stefmap(h,order,x_min,x_max,y_min,y_max,cell_size)
            rospy.sleep(0.2)
        
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e 

