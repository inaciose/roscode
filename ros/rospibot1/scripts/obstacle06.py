#!/usr/bin/env python

import time
from math import sin, cos, pi
import rospy
#from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

from array import *

class Obstacle:
     #runtime parameters
    _usRangeData = [[0,0.15], [0,0.15], [0,0.25], [0,0.25], [0,0.25], [0,0.15], [0,0.15]]
    _obstacle_detected_maxcount = 2 

    # main variables
    _cmd_timeout = 0
    _twistVelmsg = 0
    _encl = 0
    _encr = 0
    _encl_target = 0
    _encr_target = 0
    _motion_state = 0
    _motion_state_internal = 0
    _cmd_dist = 0
    _obstacle_detected = 0
    _obstacle_detected_count = 0

    def __init__(self):
        rospy.init_node("obstacle_avoid_simple")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        self.rate = rospy.get_param('~rate',50.0) 
        self.ticks_meter = float(rospy.get_param('ticks_meter', 8988))
        self.base_width = float(rospy.get_param('~base_width', 0.2))

	# ros topic subscribers
        rospy.Subscriber("ultrasound0", Range, self.cbUsRange0)
        rospy.Subscriber("ultrasound1", Range, self.cbUsRange1)
        rospy.Subscriber("ultrasound2", Range, self.cbUsRange2)
        rospy.Subscriber("ultrasound3", Range, self.cbUsRange3)
        rospy.Subscriber("ultrasound4", Range, self.cbUsRange4)
        rospy.Subscriber("ultrasound5", Range, self.cbUsRange5)
        rospy.Subscriber("ultrasound6", Range, self.cbUsRange6)
        rospy.Subscriber("encl", Int32, self.cbEncl)
        rospy.Subscriber("encr", Int32, self.cbEncr)
	rospy.Subscriber("motion_state", Int8, self.cbMotionState1)

        # ros topic publishers
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_cmd_dist = rospy.Publisher('cmd_dist', Float32, queue_size=10)

	# set cmd_vel publisher variable
        self._twistVelmsg = Twist()
        self._twistVelmsg.linear.x = 0
        self._twistVelmsg.linear.y = 0
        self._twistVelmsg.linear.z = 0
        self._twistVelmsg.angular.x = 0
        self._twistVelmsg.angular.y = 0
        self._twistVelmsg.angular.z = 0

	# control variables
	self._cmd_dist = 0
	self._motion_state = 0
	self._motion_state_internal = 0
	self._obstacle_detected = 0

    def robotMotionStop(self):
        rospy.loginfo("-D- STOP")
	# send cmd_vel topic message
        self._twistVelmsg.linear.x = 0
        self._twistVelmsg.angular.z = 0
        #rospy.loginfo("-D- publish stop: %s" % self._twistVelms")
        self.pub_cmd_vel.publish(self._twistVelmsg)
	# set control
	self._motion_state_internal = 0

    def robotMotionStraight(self, dist):
	rospy.loginfo("-D- GO STRAIGHT")
	self._cmd_dist = dist
	# send cmd_dist topic message
	if(dist > 0):
	    self.pub_cmd_dist.publish(self._cmd_dist)
	    time.sleep(0.01)
        # send cmd_vel topic message
        self._twistVelmsg.linear.x = 0.1
        self._twistVelmsg.angular.z = 0
        self.pub_cmd_vel.publish(self._twistVelmsg)
	# set control
        if(self._cmd_dist >= 0):
            rospy.loginfo("-D- GO STRAIGHT FORWARD: %f" % dist)
            self._motion_state_internal = 2
            if(self._cmd_dist != 0):
            	self._encl_target = self._encl + self._cmd_dist * self.ticks_meter
            else:
                self._encl_target = 0
        else:
            rospy.loginfo("-D- GO STRAIGHT BACKWARD: %f" % dist)
            self._motion_state_internal = 3
            self._encl_target = self._encl - self._cmd_dist * self.ticks_meter

    def robotMotionRotate(self, theta):
	self._cmd_dist = ((theta * pi) / 180) * (self.base_width / 2)
	# send cmd_dist topic message
        if(self._cmd_dist != 0):
            self.pub_cmd_dist.publish(self._cmd_dist)
            time.sleep(0.01)
        # send cmd_vel topic message
        self._twistVelmsg.linear.x = 0
	if(self._cmd_dist > 0):
            self._twistVelmsg.angular.z = 0.1
	else:
	    self._twistVelmsg.angular.z = -0.1
        self.pub_cmd_vel.publish(self._twistVelmsg)
	# set control
	if(self._cmd_dist > 0):
            rospy.loginfo("-D- ROTATE LEFT %i" % theta)
       	    self._motion_state_internal = 4
	    self._encl_target = self._encl - self._cmd_dist * self.ticks_meter
	else:
            rospy.loginfo("-D- ROTATE RIGHT %i" % theta)
	    self._motion_state_internal = 5
	    if(self._cmd_dist != 0):
	        self._encl_target = self._encl + self._cmd_dist * self.ticks_meter
	    else:
		self._encl_target = 0

    #############################################################
    def spin(self):
        rate = rospy.Rate(20)

        ###### main loop  ######
        while not rospy.is_shutdown():
	    #rospy.loginfo("-D- recycle %i %i %f %i %i %i" % (self._motion_state, self._motion_state_internal, self._cmd_dist, self._encl, self._encr, self._encl_target))

            # robot  in motion
	    if(self._motion_state > 1):

		# is run front
		if(self._motion_state == 2):
                    #rospy.loginfo("-D- running check on 2")
		    # scan sonar data array for short distances
		    for i in self._usRangeData:
			if(i[0] < i[1] and i[0] > 0):
			    self._obstacle_detected = 1

		    # apply some level to discard false readings
		    if(self._obstacle_detected == 1):
			self._obstacle_detected_count += 1
		    else:
			self._obstacle_detected_count = 0
		    self._obstacle_detected = 0

		    # detect obstacle count is above the level
		    if(self._obstacle_detected_count >= self._obstacle_detected_maxcount):
                        rospy.loginfo("-D- obstacle: %s" % self._usRangeData)
			# send stop cmd_vel
			self.robotMotionStop()
			self._obstacle_detected_count = 0

		    # overcome arduino message loose 
		    if(self._encl > self._encl_target):
                        # send stop cmd_vel
                        self.robotMotionStop()

		# is rotate left
                if(self._motion_state == 4):
                    #rospy.loginfo("-D- running check on 4")
                    # overcome arduino message loose 
                    if(self._encl < self._encl_target):
			rospy.loginfo("-D- overshot check on 4")
                        if(self._usRangeData[2][0] > 0.5 and 
                            self._usRangeData[3][0] > 0.5 and
                            self._usRangeData[4][0] > 0.5 ):
                            # send stop cmd_vel
                            self.robotMotionStop()

	    else:

	    # robot not moving
                rospy.loginfo("-D- EXTERNAL IDLE %d" % self._motion_state_internal)
		# robot not moving and ready for command
		if(self._motion_state == 0):
		    # send command to robot 
                    if(self._motion_state_internal == 0):
			rospy.loginfo("-D- INTERNAL IDLE %d" % self._motion_state)
		        time.sleep(5)
		        rospy.loginfo("-D- scan: %s" % self._usRangeData)
			i = 0
			maxdist_read = 0;
			maxdist_index = 0
			while(i < 7):
			    if(self._usRangeData[i][0] > maxdist_read):
				if(i==0):
				    if(self._usRangeData[1][0] > 0.25):
					maxdist_read = self._usRangeData[i][0]
					maxdist_index = i
                                elif(i==1):
                                    if(self._usRangeData[0][0] > 0.25):
                                        maxdist_read = self._usRangeData[i][0]
                                        maxdist_index = i
                                elif(i==2):
                                    if(self._usRangeData[3][0] > 0.25 and self._usRangeData[4][0] > 0.25 and self._usRangeData[0][0] > 0.1 and self._usRangeData[1][0] > 0.15 and self._usRangeData[5][0] > 0.15 and self._usRangeData[6][0] > 0.1):
                                        maxdist_read = self._usRangeData[i][0]
                                        maxdist_index = i
                                elif(i==3):
                                    if(self._usRangeData[2][0] > 0.25 and self._usRangeData[4][0] > 0.25 and self._usRangeData[0][0] > 0.1 and self._usRangeData[1][0] > 0.15 and self._usRangeData[5][0] > 0.15 and self._usRangeData[6][0] > 0.1):
                                        maxdist_read = self._usRangeData[i][0]
                                        maxdist_index = i
                                elif(i==4):
                                    if(self._usRangeData[2][0] > 0.25 and self._usRangeData[3][0] > 0.25 and self._usRangeData[0][0] > 0.1 and self._usRangeData[1][0] > 0.15 and self._usRangeData[5][0] > 0.15 and self._usRangeData[6][0] > 0.1):
                                        maxdist_read = self._usRangeData[i][0]
                                        maxdist_index = i
                                elif(i==5):
                                    if(self._usRangeData[6][0] > 0.25):
                                        maxdist_read = self._usRangeData[i][0]
                                        maxdist_index = i
                                elif(i==6):
                                    if(self._usRangeData[5][0] > 0.25):
                                        maxdist_read = self._usRangeData[i][0]
                                        maxdist_index = i

			    i += 1

			rospy.loginfo("-D- scan: %i %f" % (maxdist_index, maxdist_read))

			if(maxdist_index == 2 or maxdist_index == 3 or maxdist_index == 4):
			    # go front
			    self.robotMotionStraight(0.5)

			if(maxdist_index == 0):
			    # rotate left 90
			    self.robotMotionRotate(90)

                        if(maxdist_index == 1):
                            # rotate left 45
                            self.robotMotionRotate(45)

                        if(maxdist_index == 5):
                            # rotate right 45
                            self.robotMotionRotate(-45)

                        if(maxdist_index == 6):
                            # rotate right 90
                            self.robotMotionRotate(-90)

			#set control
			_cmd_timeout = time.time() + 5

		    else:
			# check command sent 
			if(time.time() >  _cmd_timeout):
			    # sync lost reset
			    rospy.loginfo("-D- CMD TIMEOUT RESET")
			    self._motion_state_internal = 0

                rospy.loginfo("-D- IDLE STATE 0 DONE")

	    rate.sleep()

    #############################################################
    def cbMotionState1(self,msg):
        #rospy.loginfo("-D- 0 MotionState1Callback: %s" % str(msg))
        self._motion_state = msg.data
	# set control
	if(msg.data == 0):
	    self._cmd_dist = 0
	    self._encl_target = 0
	    self._motion_state_internal = 0

    #############################################################
    def cbEncl(self,msg):
        self._encl = msg.data

    #############################################################
    def cbEncr(self,msg):
        self._encr = msg.data

    #############################################################
    def cbUsRange0(self,msg):
	self._usRangeData[0][0] = msg.range

    #############################################################
    def cbUsRange1(self,msg):
        self._usRangeData[1][0] = msg.range

    #############################################################
    def cbUsRange2(self,msg):
        self._usRangeData[2][0] = msg.range

    #############################################################
    def cbUsRange3(self,msg):
        self._usRangeData[3][0] = msg.range

    #############################################################
    def cbUsRange4(self,msg):
        self._usRangeData[4][0] = msg.range

    #############################################################
    def cbUsRange5(self,msg):
        self._usRangeData[5][0] = msg.range

    #############################################################
    def cbUsRange6(self,msg):
        self._usRangeData[6][0] = msg.range

#############################################################
if __name__ == '__main__':
    """ main """
    runme = Obstacle()
    runme.spin()
