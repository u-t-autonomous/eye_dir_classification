#!/usr/bin/env python
import rospy
from openbci.msg import BCIuVolts

def callback(data):
    global run
    global run_time
    run_time = data.stamp
    run = data.data
    print(run_time, run)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", BCIuVolts, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()