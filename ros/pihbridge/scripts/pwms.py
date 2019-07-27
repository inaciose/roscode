#!/usr/bin/env python

import sys
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist

class TwistToPWM():

    #############################################################
    def __init__(self):

        rospy.init_node("twist_to_pwm")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        rospy.on_shutdown(self.shutdownCallback)

	# define GPIO pin signals to use
	self.pinin1 = rospy.get_param("~pinin1", 23)
	self.pinin2 = rospy.get_param("~pinin2", 24)
	self.pinen1 = rospy.get_param("~pinen1", 13)

	self.pinin3 = rospy.get_param("~pinin3", 17)
	self.pinin4 = rospy.get_param("~pinin4", 27)
	self.pinen2 = rospy.get_param("~pinen2", 12)

        self.width = rospy.get_param("~base_width", 0.2)
        self.rate = rospy.get_param("~rate", 20)

        self.timeout_ticks = rospy.get_param("~timeout_ticks", 1)

	# setup GPIO pin address mode
	GPIO.setmode(GPIO.BCM)
	#GPIO.setmode(GPIO.BOARD)
	GPIO.cleanup()

	# set GPIO pins mode (int,out)
	GPIO.setup(self.pinin1, GPIO.OUT)
	GPIO.setup(self.pinin2, GPIO.OUT)
	GPIO.setup(self.pinen1, GPIO.OUT)
	GPIO.setup(self.pinin3, GPIO.OUT)
	GPIO.setup(self.pinin4, GPIO.OUT)
	GPIO.setup(self.pinen2, GPIO.OUT)

	# set GPIO pins initial value
	GPIO.output(self.pinin1, GPIO.LOW)
	GPIO.output(self.pinin2, GPIO.LOW)
	GPIO.output(self.pinin3, GPIO.LOW)
	GPIO.output(self.pinin4, GPIO.LOW)
	self.pwm1 = GPIO.PWM(self.pinen1, 60)
	self.pwm1.start(0)
	self.pwm2 = GPIO.PWM(self.pinen2, 60)
	self.pwm2.start(0)

	# ros topic subscriber
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)

	# main speed speed variables
	self.dx = 0
	self.dr = 0
	self.dy = 0
        self.left = 0
        self.right = 0

	# arbitrary constant factor
	self.pwmK = 20

    #############################################################
    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        ###### main loop  ######
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    #############################################################
    def spinOnce(self):

	# calculate differential speed
        self.right = 1.0 * self.dx + self.dr * self.width / 2
        self.left = 1.0 * self.dx - self.dr * self.width / 2

	# set m/s and rad/s to arbitrary pwm value
	left_tmp = self.left * self.pwmK
	right_tmp = self.right * self.pwmK

	rospy.loginfo("calculated: (%f, %f)", left_tmp, right_tmp)

	if(left_tmp > 100):
	    left_tmp = 100
        if(right_tmp > 100):
            right_tmp = 100

	# send left
	if(self.left > 0):
	    self.motorLeftForward(abs(left_tmp))
	    rospy.loginfo("self.motorLeftForward %f", left_tmp) 
	elif(self.left < 0):
	    self.motorLeftBackward(abs(left_tmp))
            rospy.loginfo("self.motorLeftBackward %f", abs(left_tmp))
 
	else:
            self.motorLeftIdle(0)

	# send right
        if(self.right > 0):
            self.motorRightForward(abs(right_tmp))
	    rospy.loginfo("self.motorRightForward %f", right_tmp)
        elif(self.right < 0):
            self.motorRighttBackward(abs(right_tmp))
	    rospy.loginfo("self.motorRightBackward %f", abs(right_tmp))
        else:
            self.motorRightIdle(0)

	# rospy.loginfo("sending: (%d, %d)", left_tmp, right_tmp)
        self.ticks_since_target += 1

    #############################################################
    def shutdownCallback(self):
	rospy.loginfo("Clean shutdown")
	GPIO.cleanup()

    #############################################################
    def motorLeftForward(self,pwm):
	GPIO.output(self.pinin1, GPIO.HIGH)
        GPIO.output(self.pinin2, GPIO.LOW)
	self.pwm1.ChangeDutyCycle(pwm)

    #############################################################
    def motorRightForward(self,pwm):
        GPIO.output(self.pinin3, GPIO.HIGH)
        GPIO.output(self.pinin4, GPIO.LOW)
        self.pwm2.ChangeDutyCycle(pwm)

    #############################################################
    def motorLeftBackward(self,pwm):
        GPIO.output(self.pinin1, GPIO.LOW)
        GPIO.output(self.pinin2, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(pwm)

    #############################################################
    def motorRightBackward(self,pwm):
        GPIO.output(self.pinin3, GPIO.LOW)
        GPIO.output(self.pinin4, GPIO.HIGH)
        self.pwm2.ChangeDutyCycle(pwm)

    #############################################################
    def motorLeftStop(self,pwm):
        GPIO.output(self.pinin1, GPIO.HIGH)
        GPIO.output(self.pinin2, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(pwm)

    #############################################################
    def motorRightStop(self,pwm):
        GPIO.output(self.pinin3, GPIO.HIGH)
        GPIO.output(self.pinin4, GPIO.HIGH)
        self.pwm2.ChangeDutyCycle(pwm)

    #############################################################
    def motorLeftIdle(self,pwm):
        GPIO.output(self.pinin1, GPIO.LOW)
        GPIO.output(self.pinin2, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(pwm)

    #############################################################
    def motorRightIdle(self,pwm):
        GPIO.output(self.pinin3, GPIO.LOW)
        GPIO.output(self.pinin4, GPIO.LOW)
        self.pwm2.ChangeDutyCycle(pwm)

    #############################################################
    def twistCallback(self,msg):
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

#############################################################
if __name__ == '__main__':
    twistToPWM = TwistToPWM()
    twistToPWM.spin()
