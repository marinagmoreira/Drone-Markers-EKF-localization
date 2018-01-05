#!/usr/bin/env python  
import roslib
roslib.load_manifest('marina')
import rospy
import math
import numpy
import time
import matplotlib
from matplotlib import pyplot as plt

import std_msgs.msg
from marina.msg import error_msg



ini_time=0

plt.ion()
plt.show()


def VisualizeData(msg):
	
	curr_time=time.time()

	#gets data from topic
	mod_att_error=msg.error_att
	mod_pos_error=msg.error_pos


	print("message",mod_att_error,mod_pos_error)

	# plots *******************************************************************

	# defines x axis
	if (curr_time - ini_time) > 5 :
		x_axis_start = curr_time - ini_time - 5
		x_axis_end = curr_time - ini_time

	else:
		x_axis_start = 0
		x_axis_end = 5

		print("x_axis_start",x_axis_start, "x_axis_end",x_axis_end)



	# plot attitude error-----------------------------------------------------------

	# open figure
	plt.figure(201, figsize=(7, 4), dpi=80, facecolor='w', edgecolor='k')

	# writes axix to figure
	plt.axis([x_axis_start, x_axis_end, 0, 20])

	# defines window position
	mgr = plt.get_current_fig_manager()
	mgr.canvas.manager.window.wm_geometry("+%d+%d" % (0, 400))

	# plots the data
	plt.plot(curr_time-ini_time, mod_att_error,'o')
	plt.draw()
	plt.pause(0.000001)
	plt.show()
	curr_time=time.time()


	# plot position error-------------------------------------------------------

	# open figure
	plt.figure(200, figsize=(7, 4), dpi=80, facecolor='w', edgecolor='k')

	# writes axix to figure
	plt.axis([x_axis_start, x_axis_end, 0, 1])

	# defines window position
	mgr = plt.get_current_fig_manager()
	mgr.canvas.manager.window.wm_geometry("+%d+%d" % (0, 0))

	# plots the data
	plt.plot(curr_time-ini_time, mod_pos_error, 'o')
	plt.draw()
	plt.pause(0.000001)
	plt.show()





'''******************************************************************************
# Main Function:
# 	Initiates kalman node, Subscribes totopics and calls Kalman
******************************************************************************'''
if __name__ == '__main__':

	#Initialization
	rospy.init_node('visio')
	ini_time=time.time();


	#Interrupts to subscribers 
	rospy.Subscriber('/error',
                      error_msg,
                      VisualizeData)





	rospy.spin()



