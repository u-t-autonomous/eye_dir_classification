#!/usr/bin/env python

import rospy
from openbci.msg import BCIuVolts
from numpy import sign
from time import sleep
import sys
import psychopy 

def talker():
	pub = rospy.Publisher('chatter', BCIuVolts, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1)
	msg = BCIuVolts()
	
	input1 = 0
	while not rospy.is_shutdown():
		while input1 != -1:
			msg.data = []
			stamp = rospy.get_rostime()
			sleep(1)
			input1 = int(input("Enter a number: "))
			msg.stamp = stamp
			msg.data.append(input1)
			pub.publish(msg)
		

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

