// simple sample for an c++ Float32MultiArray publisher

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Array_pub");
    ros::NodeHandle nh;
 
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("chatter", 1000);
 
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::Float32MultiArray msg;
        msg.data.push_back(1.0);//自己写的，可行
        msg.data.push_back(2.0);
        msg.data.push_back(3.0);
        msg.data.push_back(4.0);
 
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
