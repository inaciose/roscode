#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <wiringPi.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#define ELPIN1 25
#define ELPIN2 27
#define ERPIN1 28
#define ERPIN2 29

volatile long elpos;
volatile uint8_t elstate;

volatile long erpos;
volatile uint8_t erstate;

void elpin_isr(void) {
    uint8_t p1val = digitalRead(ELPIN1);
    uint8_t p2val = digitalRead(ELPIN2);
    uint8_t s = elstate & 3;
    if (p1val) s |= 4;
    if (p2val) s |= 8;
    elstate = (s >> 2);

    switch (s) {
        case 1: case 7: case 8: case 14:
            elpos++;
            return;
        case 2: case 4: case 11: case 13:
            elpos--;
            return;
        case 3: case 12:
            elpos += 2;
            return;
        case 6: case 9:
            elpos -= 2;
            return;
    }
}

void erpin_isr(void) {
    uint8_t p1val = digitalRead(ERPIN1);
    uint8_t p2val = digitalRead(ERPIN2);
    uint8_t s = erstate & 3;
    if (p1val) s |= 4;
    if (p2val) s |= 8;
    erstate = (s >> 2);

    switch (s) {
        case 1: case 7: case 8: case 14:
            erpos--;
            return;
        case 2: case 4: case 11: case 13:
            erpos++;
            return;
        case 3: case 12:
            erpos -= 2;
            return;
        case 6: case 9:
            erpos += 2;
            return;
    }
}

int main(int argc, char **argv) {

    //setup ros
    ros::init(argc, argv, "encoders");
    ros::NodeHandle nh;

    // setup ros publishers
    ros::Publisher el_pub = nh.advertise<std_msgs::Int32>("encl", 1000);
    ros::Publisher er_pub = nh.advertise<std_msgs::Int32>("encr", 1000);

    // setup other ros variables
    ros::Rate loop_rate(100);

    // setup the wiringPi library
    if (wiringPiSetup() < 0) {
	ROS_INFO("wiringPiSetup");
        exit(EXIT_FAILURE);
    }

    // setup left encoder pins
    pinMode (ELPIN1,  INPUT);
    pinMode (ELPIN2,  INPUT);
    pullUpDnControl(ELPIN1, PUD_UP);
    pullUpDnControl(ELPIN2, PUD_UP);

    // setup right encoder pins
    pinMode (ERPIN1,  INPUT);
    pinMode (ERPIN2,  INPUT);
    pullUpDnControl(ERPIN1, PUD_UP);
    pullUpDnControl(ERPIN2, PUD_UP);

    // setup left encoder isr
    if ( wiringPiISR (ELPIN1, INT_EDGE_BOTH, &elpin_isr) < 0 ) {
	ROS_INFO("wiringPiISRl1");
        exit(EXIT_FAILURE);
    }

    if ( wiringPiISR (ELPIN2, INT_EDGE_BOTH, &elpin_isr) < 0 ) {
        ROS_INFO("wiringPiISRl2");
        exit(EXIT_FAILURE);
    }

    // setup right encoder isr
    if ( wiringPiISR (ERPIN1, INT_EDGE_BOTH, &erpin_isr) < 0 ) {
        ROS_INFO("wiringPiISRr1");
        exit(EXIT_FAILURE);
    }

    if ( wiringPiISR (ERPIN2, INT_EDGE_BOTH, &erpin_isr) < 0 ) {
        ROS_INFO("wiringPiISRr2");
        exit(EXIT_FAILURE);
    }

    // give feedback after setup complete
    ROS_INFO("diff encoders setup done");

    // main loop
    while ( ros::ok() ) {
	std_msgs::Int32 msg;
        // publish left encoder ros topic
	msg.data = elpos;
	el_pub.publish(msg);
        // publish right encoder  ros topic
        msg.data = erpos;
        er_pub.publish(msg);
	// spin & sleep
	ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
}
