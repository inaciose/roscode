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
#define PI 3.141592653

#define PWM_K 20.0
#define ENCODER_PUBLISHER_TIMER 0.02

// units are m, m/s, radian/s
double wheel_encoder_pulses = 1920.0;
double wheel_rad = 0.034, wheel_sep = 0.200;
// stored for fast calculation
double wheel_per = 2 * PI * wheel_rad;
double distancePerPulse = wheel_per / wheel_encoder_pulses;
double pulses_per_m = 1.0 / wheel_per * wheel_encoder_pulses;

// left encoder variables
volatile long encoderLeftPulses;
volatile uint8_t encoderLeftState;

// right encoder variables
volatile long encoderRightPulses;
volatile uint8_t encoderRightState;

// pwm & pid variables
double cmd_vel_x, cmd_vel_z;
double leftMotorSpeedRequest,rightMotorSpeedRequest;

volatile long encoderLeftSpeedPidPulses;
volatile long encoderRightSpeedPidPulses;

double leftSpeedPidInput, rightSpeedPidInput;
double leftSpeedPidSetPoint, rightSpeedPidSetPoint;
int leftSpeedPidSetPointDirection, rightSpeedPidSetPointDirection;

double pidTimer;

//PID constants
#define PID_SAMPLE_TIME 0.050
double kpLeft = 0.2;
double kiLeft = 0.001;
double kdLeft = 0;

double kpRight = 0.2;
double kiRight = 0.001;
double kdRight = 0;

// control variables
int msgFlag = 0;
double encoderPublisherTimer;

void encoderLeftPinIsr(void) {
    uint8_t p1val = digitalRead(ELPIN1);
    uint8_t p2val = digitalRead(ELPIN2);
    uint8_t s = encoderLeftState & 3;
    if (p1val) s |= 4;
    if (p2val) s |= 8;
    encoderLeftState = (s >> 2);

    switch (s) {
        case 1: case 7: case 8: case 14:
            encoderLeftPulses++;
	    encoderLeftSpeedPidPulses++;
            return;
        case 2: case 4: case 11: case 13:
            encoderLeftPulses--;
	    encoderLeftSpeedPidPulses--;
            return;
        case 3: case 12:
            encoderLeftPulses += 2;
	    encoderLeftSpeedPidPulses += 2;
            return;
        case 6: case 9:
            encoderLeftPulses -= 2;
	    encoderLeftSpeedPidPulses -= 2;
            return;
    }
}

void encoderRightPinIsr(void) {
    uint8_t p1val = digitalRead(ERPIN1);
    uint8_t p2val = digitalRead(ERPIN2);
    uint8_t s = encoderRightState & 3;
    if (p1val) s |= 4;
    if (p2val) s |= 8;
    encoderRightState = (s >> 2);

    switch (s) {
        case 1: case 7: case 8: case 14:
            encoderRightPulses--;
	    encoderRightSpeedPidPulses--;
            return;
        case 2: case 4: case 11: case 13:
            encoderRightPulses++;
	    encoderRightSpeedPidPulses++;
            return;
        case 3: case 12:
            encoderRightPulses -= 2;
	    encoderRightSpeedPidPulses -= 2;
            return;
        case 6: case 9:
            encoderRightPulses += 2;
	    encoderRightSpeedPidPulses += 2;
            return;
    }
}

void motorLeftForward(int pwm) {
    digitalWrite(PININ1, HIGH);
    digitalWrite(PININ2, LOW);
    softPwmWrite(PINEN1, pwm);
    //ROS_INFO("motorLeftForward: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void motorRightForward(int pwm) {
    digitalWrite(PININ3, HIGH);
    digitalWrite(PININ4, LOW);
    softPwmWrite(PINEN2, pwm);
    //ROS_INFO("motorRightForward: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}


void motorLeftBackward(int pwm) {
    digitalWrite(PININ1, LOW);
    digitalWrite(PININ2, HIGH);
    softPwmWrite(PINEN1, pwm);
    //ROS_INFO("motorLeftBackward: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void motorRightBackward(int pwm) {
    digitalWrite(PININ3, LOW);
    digitalWrite(PININ4, HIGH);
    softPwmWrite(PINEN2, pwm);
    //ROS_INFO("motorRightBackward: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void motorLeftIdle(int pwm) {
    digitalWrite(PININ1, LOW);
    digitalWrite(PININ2, LOW);
    softPwmWrite(PINEN1, pwm);
    //ROS_INFO("motorLeftIdle: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void motorRightIdle(int pwm) {
    digitalWrite(PININ3, LOW);
    digitalWrite(PININ4, LOW);
    softPwmWrite(PINEN2, pwm);
    //ROS_INFO("motorRightIdle: %f %f %d", cmd_vel_x, cmd_vel_z, pwm);
}

void twistCallback(const geometry_msgs::Twist& msg) {
    ROS_INFO("twistCallback: %f, %f", msg.linear.x, msg.angular.z);
    cmd_vel_x = msg.linear.x;
    cmd_vel_z = msg.angular.z;
    msgFlag = 1;
}

void setupEncoders(void) {
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
    if ( wiringPiISR (ELPIN1, INT_EDGE_BOTH, &encoderLeftPinIsr) < 0 ) {
        ROS_INFO("wiringPiISRl1 error: exit");
        exit(EXIT_FAILURE);
    }

    if ( wiringPiISR (ELPIN2, INT_EDGE_BOTH, &encoderLeftPinIsr) < 0 ) {
        ROS_INFO("wiringPiISRl2 error: exit");
        exit(EXIT_FAILURE);
    }

    // setup right encoder isr
    if ( wiringPiISR (ERPIN1, INT_EDGE_BOTH, &encoderRightPinIsr) < 0 ) {
        ROS_INFO("wiringPiISRr1");
        exit(EXIT_FAILURE);
    }

    if ( wiringPiISR (ERPIN2, INT_EDGE_BOTH, &encoderRightPinIsr) < 0 ) {
        ROS_INFO("wiringPiISRr2");
        exit(EXIT_FAILURE);
    }  
}

void setupMotors(void) {
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
}

double computeLeftPID(double inp) {
        unsigned long currentTime;
	static unsigned long previousTime = millis();
        double elapsedTime;
	double error;
	double rateError;
	static double lastError = 0;
	static double cumError = 0;
	
        currentTime = millis();
        elapsedTime = (double)(currentTime - previousTime);
        
        error = leftSpeedPidSetPoint - inp; // compute proporcional
        cumError += error * elapsedTime; // compute integral
        rateError = (error - lastError)/elapsedTime; // compute derivative

        double out = kpLeft*error + kiLeft*cumError + kdLeft*rateError;

	ROS_INFO("PID %f %f %f %f %f %f", error, cumError, rateError, elapsedTime, inp, out);
	
	if(out > 100) out = 100;
	if(out < 0) out = 0;
	
        lastError = error;
        previousTime = currentTime;

        return out;
}

double computeRightPID(double inp) {
        unsigned long currentTime;
	static unsigned long previousTime = millis();
        double elapsedTime;
	double error;
	double rateError;
	static double lastError = 0;
	static double cumError = 0;
	
        currentTime = millis();
        elapsedTime = (double)(currentTime - previousTime);
        
        error = rightSpeedPidSetPoint - inp; // compute proporcional
        cumError += error * elapsedTime; // compute integral
        rateError = (error - lastError)/elapsedTime; // compute derivative

        double out = kpRight*error + kiRight*cumError + kdRight*rateError;

	if(out > 100) out = 100;
	if(out < 0) out = 0;

        lastError = error;
        previousTime = currentTime;

        return out;
}


int main(int argc, char **argv) {

    //setup ros
    ros::init(argc, argv, "diffbotpid");
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

    // setup motors & encoders
    setupEncoders();
    setupMotors();

    // setup pid
    leftSpeedPidSetPoint = 0;
    rightSpeedPidSetPoint = 0;
    pidTimer = ros::Time::now().toSec();

    // give feedback after setup complete
    ROS_INFO("diffbot setup done");

    // main loop
    while ( ros::ok() ) {
        std_msgs::Int32 msg;
	double ltmp, rtmp;

	if(ros::Time::now().toSec() > pidTimer) {
	    leftSpeedPidInput = abs(encoderLeftSpeedPidPulses);
	    rightSpeedPidInput = abs(encoderRightSpeedPidPulses);
	    
	    ltmp = computeLeftPID(leftSpeedPidInput);
	    rtmp = computeRightPID(rightSpeedPidInput);

	    encoderLeftSpeedPidPulses = 0;
	    encoderRightSpeedPidPulses = 0;

	    ROS_INFO("PID update %f %f %f %f %f %f", leftSpeedPidInput, rightSpeedPidInput, leftSpeedPidSetPoint, rightSpeedPidSetPoint, ltmp, rtmp);
	    
            // set left motor direction and velocity
  	    if( leftMotorSpeedRequest > 0) {
	        motorLeftForward(abs(ltmp));
	    } else if(leftMotorSpeedRequest < 0) {
	        motorLeftBackward(abs(ltmp));
	    } else {
	        motorLeftIdle(0);
	    }

	    // set right motor direction and  velocity
            if(rightMotorSpeedRequest > 0) {
                motorRightForward(abs(rtmp));
            } else if(rightMotorSpeedRequest < 0) {
                motorRightBackward(abs(rtmp));
            } else {
                motorRightIdle(0);
            }
	    
	    pidTimer = ros::Time::now().toSec() + PID_SAMPLE_TIME;
	}
	  
	// update motors on new cmd_vel message
        if(msgFlag) {
	    // calculate motors speed
	    //leftMotorSpeedRequest = 1.0 * cmd_vel_x - cmd_vel_z * wheel_sep / 2;
	    //rightMotorSpeedRequest = 1.0 * cmd_vel_x + cmd_vel_z * wheel_sep / 2;

	    leftMotorSpeedRequest = (cmd_vel_x/wheel_per) - ((cmd_vel_z*wheel_sep)/wheel_per);
            rightMotorSpeedRequest = (cmd_vel_x/wheel_per) + ((cmd_vel_z*wheel_sep)/wheel_per);

	    ROS_INFO("conv1 vel: %f %f", leftMotorSpeedRequest, rightMotorSpeedRequest);

	    rightMotorSpeedRequest = rightMotorSpeedRequest * wheel_encoder_pulses / 1 * PID_SAMPLE_TIME;
	    leftMotorSpeedRequest = leftMotorSpeedRequest * wheel_encoder_pulses / 1 * PID_SAMPLE_TIME;

	    ROS_INFO("pulses vel: %f %f", leftMotorSpeedRequest, rightMotorSpeedRequest);
	    
	    // set left motors direction and speed
	    if(leftMotorSpeedRequest > 0) {
	      leftSpeedPidSetPointDirection = 1;
	      leftSpeedPidSetPoint = round(leftMotorSpeedRequest);
	    } else if(leftMotorSpeedRequest < 0) {
	      leftSpeedPidSetPointDirection = -1;
	      leftSpeedPidSetPoint = round(abs(leftMotorSpeedRequest));
	    } else {
	      leftSpeedPidSetPointDirection = 0;
	      leftSpeedPidSetPoint = round(leftMotorSpeedRequest);
	    }	    

	    // set right motors direction and speed
	    if(rightMotorSpeedRequest > 0) {
	      rightSpeedPidSetPointDirection = 1;
	      rightSpeedPidSetPoint = round(rightMotorSpeedRequest);
	    } else if(rightMotorSpeedRequest < 0) {
	      rightSpeedPidSetPointDirection = -1;
	      rightSpeedPidSetPoint = round(abs(rightMotorSpeedRequest));
	    } else {
	      rightSpeedPidSetPointDirection = 0;
	      rightSpeedPidSetPoint = 0;
	    }

	    ROS_INFO("Setpoint %f %f %f %f", leftMotorSpeedRequest, rightMotorSpeedRequest, leftSpeedPidSetPoint, rightSpeedPidSetPoint);

	    msgFlag = 0;
        }

	// publish encoders pulse count on timer
        if(ros::Time::now().toSec() > encoderPublisherTimer) {
            // publish left encoder ros topic
            msg.data = encoderLeftPulses;
            el_pub.publish(msg);
            // publish right encoder  ros topic
            msg.data = encoderRightPulses;
            er_pub.publish(msg);
	    encoderPublisherTimer = ros::Time::now().toSec() + ENCODER_PUBLISHER_TIMER;
	}

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
