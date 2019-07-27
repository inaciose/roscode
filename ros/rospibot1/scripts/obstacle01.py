#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class Obstacle:
    def __init__(self):
        rospy.init_node("obstacle_avoid_simple")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        self.rate = rospy.get_param('~rate',10.0) 
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)

        rospy.Subscriber("ultrasound3", Range, self.rangeCallback)
        #self.pub = rospy.Publisher('cmd_vel1', Twist, queue_size=10)
        self.pub = rospy.Publisher('cmd_vel1', Float32, queue_size=10)

        self.read = 0

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
        #rospy.loginfo("publishing: (%f)", self.read) 
        self.pub.publish(self.read)
        self.ticks_since_target += 1

    #############################################################
    def rangeCallback(self,msg):
        #rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.read = msg.range

#############################################################
if __name__ == '__main__':
    """ main """
    runme = Obstacle()
    runme.spin()
