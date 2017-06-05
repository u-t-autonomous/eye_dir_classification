#!/usr/bin/env python

import rospy
from openbci.msg import BCIuVolts
from numpy import sign
from time import sleep
import sys
import psychopy 

def filereader():
	pub = rospy.Publisher('chatter', BCIuVolts, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(100)
	msg = BCIuVolts()
	f = open('/home/siddarthkaki/Desktop/EOG_data_files_2/5-23-17/data_main_2.txt','r')
	current_line = f.readline()
	
	while not rospy.is_shutdown():
		while current_line != "":
			msg.data = []
			stamp = rospy.get_rostime()
			current_line = f.readline()
			new_sample = current_line
			new_sample = (current_line[21:28])
			try:
				new_sample = float(new_sample)
				msg.stamp = stamp
				msg.data.append(new_sample)
				pub.publish(msg)
				rate.sleep()
			except:
				pass
		print('final data sent')		
if __name__ == '__main__':
	try:
		# sleep(12)
		filereader()
	except rospy.ROSInterruptException:
		pass

