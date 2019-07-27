#!/usr/bin/env python

import time
import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

from array import *

class Obstacle:
    _usRangeData = [[0,0.15], [0,0.15], [0,0.25], [0,0.25], [0,0.25], [0,0.15], [0,0.15]]
    _twistVelmsg = 0
    _motion_state = 0
    _obstacle_detected = 0

    def __init__(self):
        rospy.init_node("obstacle_avoid_simple")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        self.rate = rospy.get_param('~rate',50.0) 
        #self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)

	# ros topic subscribers
        rospy.Subscriber("ultrasound0", Range, self.cbUsRange0)
        rospy.Subscriber("ultrasound1", Range, self.cbUsRange1)
        rospy.Subscriber("ultrasound2", Range, self.cbUsRange2)
        rospy.Subscriber("ultrasound3", Range, self.cbUsRange3)
        rospy.Subscriber("ultrasound4", Range, self.cbUsRange4)
        rospy.Subscriber("ultrasound5", Range, self.cbUsRange5)
        rospy.Subscriber("ultrasound6", Range, self.cbUsRange6)

        # ros topic publishers
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# set cmd_vel publisher variable
        self._twistVelmsg = Twist()
        self._twistVelmsg.linear.x = 0
        self._twistVelmsg.linear.y = 0
        self._twistVelmsg.linear.z = 0
        self._twistVelmsg.angular.x = 0
        self._twistVelmsg.angular.y = 0
        self._twistVelmsg.angular.z = 0

	# subscribers variables
	# on class declaration

	# control variables
	self._motion_state = 0
	self._obstacle_detected = 0

    #############################################################
    def spin(self):
        #r = rospy.Rate(10)
        #self.ticks_since_range = self.timeout_ticks

        ###### main loop  ######
        while not rospy.is_shutdown():

	    if(self._motion_state != 0):
		if(self._motion_state == 2):
                    #rospy.loginfo("-D- running check")
		    for i in self._usRangeData:
			if(i[0] < i[1]):
			    self._obstacle_detected = 1

		    if(self._obstacle_detected == 1):
                        #rospy.loginfo("-D- obstacle: %s" % self._usRangeData)
			# prepare topic message
                        self._twistVelmsg.linear.x = 0
                        self._twistVelmsg.angular.z = 0
                	#rospy.loginfo("-D- publish stop: %s" % self._twistVelmsg)
			# send topic message
			self.pub.publish(self._twistVelmsg)
			# update control
			self._obstacle_detected = 0
			self._motion_state = 0
	    else:
                rospy.loginfo("-D- idle")
                # prepare topic message
                self._twistVelmsg.linear.x = 0
                self._twistVelmsg.angular.z = 0
	        #rospy.loginfo("-D- publish go front: %s" % self._twistVelmsg)
                # send topic message
                self.pub.publish(self._twistVelmsg)
                # update control
                self._motion_state = 2

            #rospy.loginfo("-D- state/obstacle: (%s, %s)" % (self._motion_state, self._obstacle_detected))
	    #r.sleep()
	    time.sleep(0.01)

    #############################################################
    def cbUsRange0(self,msg):
        #rospy.loginfo("-D- 0 twistCallback: %s" % str(msg))
        #self.ticks_since_range = 0
	self._usRangeData[0][0] = msg.range

    #############################################################
    def cbUsRange1(self,msg):
        #rospy.loginfo("-D- 1 twistCallback: %s" % str(msg))
        #self.ticks_since_range = 0
        self._usRangeData[1][0] = msg.range

    #############################################################
    def cbUsRange2(self,msg):
        #rospy.loginfo("-D- 2 twistCallback: %s" % str(msg))
        #self.ticks_since_range = 0
        self._usRangeData[2][0] = msg.range

    #############################################################
    def cbUsRange3(self,msg):
        #rospy.loginfo("-D- 3 twistCallback: %s" % str(msg))
        #self.ticks_since_range = 0
        self._usRangeData[3][0] = msg.range

    #############################################################
    def cbUsRange4(self,msg):
        #rospy.loginfo("-D- 4 twistCallback: %s" % str(msg))
        #self.ticks_since_range = 0
        self._usRangeData[4][0] = msg.range

    #############################################################
    def cbUsRange5(self,msg):
        #rospy.loginfo("-D- 5 twistCallback: %s" % str(msg))
        #self.ticks_since_range = 0
        self._usRangeData[5][0] = msg.range

    #############################################################
    def cbUsRange6(self,msg):
        #rospy.loginfo("-D- 6 twistCallback: %s" % str(msg))
        #self.ticks_since_range = 0
        self._usRangeData[6][0] = msg.range

#############################################################
if __name__ == '__main__':
    """ main """
    runme = Obstacle()
    runme.spin()
