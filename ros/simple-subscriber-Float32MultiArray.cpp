// simple sample for an c++ Float32MultiArray subscriber

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
 
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("I heard: [%f],[%f],[%f],[%f]", msg->data.at(0),msg->data.at(1),msg->data.at(2),msg->data.at(3));
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Array_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}
