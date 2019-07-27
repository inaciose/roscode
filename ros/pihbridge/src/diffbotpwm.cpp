#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

// encoder pins
#define ELPIN1 25
#define ELPIN2 27
#define ERPIN1 28
#define ERPIN2 29

// h-bridge pins
#define PININ1 4
#define PININ2 5
#define PINEN1 23
#define PININ3 0
#define PININ4 2
#define PINEN2 26

// phisics
#define BASE_WIDTH 0.2
#define PWM_K 20.0
#define ENCODER_PUBLISHER_TIMER 0.02

// left encoder variables
volatile long elpos;
volatile uint8_t elstate;

// right encoder variables
volatile long erpos;
volatile uint8_t erstate;

// pwm variables
double cmd_vel_x, cmd_vel_z;
double mlpwm, mrpwm;

// control variables
int msgFlag = 0;
double encoderPublisherTimer;

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

void motorLeftForward(int pwm) {
    digitalWrite(PININ1, HIGH);
    digitalWrite(PININ2, LOW);
    softPwmWrite(PINEN1, pwm);
    ROS_INFO("motorLeftForward: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void motorRightForward(int pwm) {
    digitalWrite(PININ3, HIGH);
    digitalWrite(PININ4, LOW);
    softPwmWrite(PINEN2, pwm);
    ROS_INFO("motorRightForward: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}


void motorLeftBackward(int pwm) {
    digitalWrite(PININ1, LOW);
    digitalWrite(PININ2, HIGH);
    softPwmWrite(PINEN1, pwm);
    ROS_INFO("motorLeftBackward: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void motorRightBackward(int pwm) {
    digitalWrite(PININ3, LOW);
    digitalWrite(PININ4, HIGH);
    softPwmWrite(PINEN2, pwm);
    ROS_INFO("motorRightBackward: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void motorLeftIdle(int pwm) {
    digitalWrite(PININ1, LOW);
    digitalWrite(PININ2, LOW);
    softPwmWrite(PINEN1, pwm);
    ROS_INFO("motorLeftIdle: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void motorRightIdle(int pwm) {
    digitalWrite(PININ3, LOW);
    digitalWrite(PININ4, LOW);
    softPwmWrite(PINEN2, pwm);
    ROS_INFO("motorRightIdle: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void twistCallback(const geometry_msgs::Twist& msg) {
    ROS_INFO("twistCallback: %f, %f", msg.linear.x, msg.angular.z);
    cmd_vel_x = msg.linear.x;
    cmd_vel_z = msg.angular.z;
    msgFlag = 1;
}

int main(int argc, char **argv) {

    //setup ros
    ros::init(argc, argv, "diffbotpwm");
    ros::NodeHandle nh;

    // setup ros publishers
    ros::Publisher el_pub = nh.advertise<std_msgs::Int32>("encl", 1000);
    ros::Publisher er_pub = nh.advertise<std_msgs::Int32>("encr", 1000);

    // setup ros subscribers
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, twistCallback);

    // setup other ros variables
    ros::Rate loop_rate(100);
    encoderPublisherTimer = ros::Time::now().toSec();

    // setup the wiringPi library
    if (wiringPiSetup() < 0) {
        ROS_INFO("wiringPiSetup error: exit");
        exit(EXIT_FAILURE);
    }

    //
    // encoders
    //

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
        ROS_INFO("wiringPiISRl1 error: exit");
        exit(EXIT_FAILURE);
    }

    if ( wiringPiISR (ELPIN2, INT_EDGE_BOTH, &elpin_isr) < 0 ) {
        ROS_INFO("wiringPiISRl2 error: exit");
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

    //
    // h-bridge
    //

    // direction pins
    pinMode (PININ1,  OUTPUT);
    pinMode (PININ2,  OUTPUT);
    pinMode (PININ3,  OUTPUT);
    pinMode (PININ4,  OUTPUT);

    digitalWrite(PININ1, LOW);
    digitalWrite(PININ2, LOW);
    digitalWrite(PININ3, LOW);
    digitalWrite(PININ4, LOW);

    // pwm pins
    if(softPwmCreate(PINEN1, 0, 100)) {
      ROS_INFO("error l");
    }

    if(softPwmCreate(PINEN2, 0, 100)) {
      ROS_INFO("error r");
    }

    // give feedback after setup complete
    ROS_INFO("diffbot setup done");

    // main loop
    while ( ros::ok() ) {
        std_msgs::Int32 msg;
	int ltmp, rtmp;

	// update motors on new cmd_vel message
        if(msgFlag) {
	    // calculate motors speed
	    mlpwm = 1.0 * cmd_vel_x - cmd_vel_z * BASE_WIDTH / 2;
	    mrpwm = 1.0 * cmd_vel_x + cmd_vel_z * BASE_WIDTH / 2;

	    ltmp = mlpwm * PWM_K;
	    rtmp = mrpwm * PWM_K;

            // set left motor direction and velocity
  	    if(mlpwm > 0) {
	        motorLeftForward(abs(ltmp));
	    } else if(mlpwm < 0) {
	        motorLeftBackward(abs(ltmp));
	    } else {
	        motorLeftIdle(0);
	    }

	    // set right motor direction and  velocity
            if(mrpwm > 0) {
                motorRightForward(abs(rtmp));
            } else if(mrpwm < 0) {
                motorRightBackward(abs(rtmp));
            } else {
                motorRightIdle(0);
            }

	    msgFlag = 0;
        }

	// publish encoders pulse count on timer
        if(ros::Time::now().toSec() > encoderPublisherTimer) {
            // publish left encoder ros topic
            msg.data = elpos;
            el_pub.publish(msg);
            // publish right encoder  ros topic
            msg.data = erpos;
            er_pub.publish(msg);
	    encoderPublisherTimer = ros::Time::now().toSec() + ENCODER_PUBLISHER_TIMER;
	}

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
