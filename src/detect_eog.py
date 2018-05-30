#!/usr/bin/env python

import rospy
from openbci.msg import BCIuVolts
from openbci.msg import Sim, Action
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from time import sleep
from stft import STFT
from movavg import MovAvg
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import itertools

class Detect():
	def __init__(self):
		# Initialize node
		rospy.init_node('detect_eog', anonymous=True)

		#Left/Right list intialization
		self.list4_1 = deque(maxlen=500)
		self.list4_detrended = []
		self.list4_2 = []


		#Up/Down list intialization
		self.list5_1 = deque(maxlen=500)
		self.list5_detrended = []
		self.list5_2 = []

		self.command_list = []

		# Subscribe
		# rospy.Subscriber("chatter", BCIuVolts, self.newSample)
		self.pub = rospy.Publisher('live_cmd', Action, queue_size=10)
		self.pub2 = rospy.Publisher('centered_new', BCIuVolts, queue_size=10)
		# self.pub3 = rospy.Publisher('live_UD_cmd', Action, queue_size=10)
		rospy.Subscriber("/openbci/eeg_channels", BCIuVolts, self.newSample)

	def newSample(self, msg):
	
		channel_data = msg.data
		self.list4_1.append(channel_data[3])
		self.list5_1.append(channel_data[4])

		if len(self.list4_1)>499:
			# print('here')
			list4_2 = self.reduce_samples_pre(list(self.list4_1))
			list5_2 = self.reduce_samples_pre(list(self.list5_1))

			
			# Left/Right Detrending
			for i in range(25, len(list4_2)):
				movavg = np.mean(list4_2[i-25:i-1])
				self.list4_detrended.append(list4_2[i]-movavg)
				movavg_2 = np.mean(list5_2[i-25:i-1])
				self.list5_detrended.append(list5_2[i]-movavg_2)

				
			self.list4_2 = []
			self.list5_2 = []
			
			list4_clf = self.reduce_samples_post(self.list4_detrended)
			list5_clf = self.reduce_samples_post(self.list5_detrended)
			self.list4_detrended = []	
			self.list5_detrended = []
			self.publish_detrended(list4_clf[-1], list5_clf[-1])

			msg = Action()
			clf_RL, mag_RL = self.clf_RL(list4_clf)
			clf_UD, mag_UD = self.clf_UD(list5_clf)

			if clf_RL == 'east' and clf_UD == 'stay' or clf_RL == 'west' and clf_UD == 'stay':
				self.command_list.append(clf_RL)
			elif clf_RL == 'stay' and clf_UD == 'north' or clf_RL == 'stay' and clf_UD == 'south':
				self.command_list.append(clf_UD)
			else:
				if mag_RL-mag_UD>30:
					self.command_list.append(clf_RL)
				elif mag_RL-mag_UD<20:
					self.command_list.append(clf_UD)
				else:
					self.command_list.append('stay')

			
			if len(self.command_list)>1000:
				command_dict = {0: 'east', 1: 'west', 2: 'north', 3: 'south'}
				eastcount = self.command_list.count('east')
				westcount = self.command_list.count('west')
				upcount = self.command_list.count('north')
				downcount = self.command_list.count('south')
				# staycount = self.command_list.count('stay')
				countlist = [eastcount, westcount, upcount, downcount]
				if sum(countlist) == 0:
					msg.rand = 'stay'
				else:
					index_max = countlist.index(max(countlist))
					msg.rand = command_dict[index_max]
				self.pub.publish(msg)
				self.command_list = []

				

	def most_common(lst):
    		return max(set(lst), key=lst.count)

	def reduce_samples_pre(self, list1):
		list_size = len(list1)
		list_size = list_size - list_size % 10 - 1
		list4_2 = []
		# print('here')
		for i in range(0, list_size, 10):
			new_entry = np.mean(list1[i:i + 9])
			list4_2.append(new_entry)
		# print len(list4_2)
		return list4_2

	def reduce_samples_post(self, list1):
		# msg = Float32()
		list4_clf = []
		sample_size = 10
		list_size = len(list1)
		list_size = (list_size - (list_size % sample_size)) - 1
		# print('here')
		for i in range(0, list_size, sample_size):
			new_entry = np.mean(list1[i:i + sample_size - 1])
			list4_clf.append(new_entry)
		return list4_clf

	def publish_detrended(self, new_entry1, new_entry2):
		msg = BCIuVolts()
		stamp = rospy.get_rostime()
		msg.data = [new_entry1, new_entry2]
		msg.stamp = stamp
		self.pub2.publish(msg)


	def clf_RL(self, list1):
		# list_size = len(list1)
		threshold_blink = 500
		threshold_east = 300
		threshold_west = -300

		west_count = sum([1 for x in list1 if x < threshold_west])

		east_count = sum([1 for x in list1 if x > threshold_east])

		blink_count = sum([1 for x in list1 if x > threshold_blink])

		if blink_count > 0:
			clf_RL = 'stay'
			mag = threshold_blink

		# condition is correct, label switched during experiment 
		elif west_count > east_count:
			clf_RL = 'east'
			mag = abs(min(list1))
		elif east_count > west_count:
			clf_RL = 'west'
			mag = max(list1)
		else:
			clf_RL = 'stay'
			mag = 0


		self.list4_clf = []
		return (clf_RL, mag)

	def clf_UD(self, list1):
		# list_size = len(list1)
		threshold_blink = 500
		threshold_up = 400
		threshold_down = -400

		down_count = sum([1 for x in list1 if x < threshold_down])

		up_count = sum([1 for x in list1 if x > threshold_up])

		blink_count = sum([1 for x in list1 if x > threshold_blink])

		if blink_count > 0:
			clf_UD = 'stay'
			mag = threshold_blink

		elif down_count > up_count:
			clf_UD = 'south'
			mag = abs(min(list1))
		elif up_count > down_count:
			clf_UD = 'north'
			mag = max(list1)
		else:
			clf_UD = 'stay'
			mag = 0
		self.list5_clf = []
		return (clf_UD, mag)


if __name__ == '__main__':
	try:
		sleep(6)
		node = Detect()
		print 'node detect'
		rospy.spin()
	except rospy.ROSInterruptException:
		pass







