#! /usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub_p = rospy.Publisher('mypoint', Float32MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	      array = [521,1314]
	      varpoint = Float32MultiArray(data=array)
        rospy.loginfo(varpoint)
	      pub_p.publish(varpoint)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
