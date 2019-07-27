#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include "include/PID_v1.h"

#define PI 3.141592653

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
//#define BASE_WIDTH 0.2
//#define PWM_K 20.0
#define ENCODER_PUBLISHER_TIMER 0.02

// kinematics config
// wheel_rad is the wheel radius
// wheel_sep is
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
volatile long encoderRightPulses;;
volatile uint8_t encoderRightState;

// pwm variables added for twist messages
double speed_lin, speed_ang;
double leftMotorSpeedRequest,rightMotorSpeedRequest;
int leftSpeedPidSetPointDirection;
int rightSpeedPidSetPointDirection;

double minPidVel = 0.40;
double maxPidVel = 1.40;

int leftMotorPwmOut = 0;
int rightMotorPwmOut = 0;

//
// pid
//

volatile long encoderLeftPulsesSpeedPID; // the number of pulses for PID
volatile long encoderLeftPulsesSteeringPID; // the number of pulses for PID, must be reset at same time
volatile long encoderRightPulsesSpeedPID; // the number of pulses PID
volatile long encoderRightPulsesSteeringPID; // the number of pulses for PID, must be reset at same time

// encoders distance targets
enum e_encoderPulsesTargetStates {ENC_TARGET_DISABLE, ENC_TARGET_READY, ENC_TARGET_ENABLED};
int encoderPulsesTargetState = ENC_TARGET_DISABLE;
long encoderLeftPulsesTarget = 0;
long encoderRightPulsesTarget = 0;
bool encoderLeftPulsesOnTarget = false;
bool encoderRightPulsesOnTarget = false;

enum e_baseMotionStates { MOTION_IDLE, MOTION_STOP, MOTION_FORWARD, MOTION_BACKWARD, MOTION_ROTATE_LEFT, MOTION_ROTATE_RIGHT, MOTION_FORWARD_CURVE_LEFT, MOTION_FORWARD_CURVE_RIGHT, MOTION_BACKWARD_CURVE_LEFT, MOTION_BACKWARD_CURVE_RIGHT};
int baseMotionState = MOTION_IDLE;

// motion idle delay timer
#define MOTION_IDLE_DELAY 120
unsigned long motion_idle_timer;

// steering PID
#define STEERING_PID_TARGET 0
#define STEERING_PID_SAMPLE_TIME 5
#define STEERING_PID_MIN_OUTPUT -100
#define STEERING_PID_MAX_OUTPUT 100
#define STEERING_PID_ADAPTATIVE_LIMIT 20
#define STEERING_PID_ADAPTATIVE_DELAY 100
double steeringPidKp0 = 0.4;
double steeringPidKi0 = 0.001;
double steeringPidKd0 = 0.0;
double steeringPidKp1 = 0.8;
double steeringPidKi1 = 0.003;
double steeringPidKd1 = 0;
bool steeringPidAdaptativeStart = false;
unsigned long steeringPidAdaptativeTimer;
double steeringPidInput = 0;
int steeringPidInputLast = 0;
double steeringPidOutput; // Power supplied to the motor PWM value.
double steeringPidSetPoint;
PID steeringPid(&steeringPidInput, &steeringPidOutput, &steeringPidSetPoint, steeringPidKp0, steeringPidKi0, steeringPidKd0, REVERSE); 
bool steeringdPidResult = false;
bool useSteeringPid = false;

#define SPEED_PID_TARGET 40
#define SPEED_PID_SAMPLE_TIME 25
#define SPEED_PID_ADAPTATIVE_LIMIT1 15
#define SPEED_PID_ADAPTATIVE_LIMIT2 30

// speed change process
#define SPEED_PID_CHANGE_DELAY 300   // time to wait before start change speed (SPEED_PID_SAMPLE_TIME * 12)

unsigned long speedPidChangeTimer;
int speedPidStepInterval = 250; // time between speed change steps
int speedPidChangeStatus = 0;

// left speed PID control
#define LEFT_SPEED_PID_MIN_OUTPUT 15
#define LEFT_SPEED_PID_MAX_OUTPUT 100
double leftSpeedPidKp0 = 10;
double leftSpeedPidKi0 = 3;
double leftSpeedPidKd0 = 0;

double leftSpeedPidKp1 = leftSpeedPidKp0;
double leftSpeedPidKi1 = leftSpeedPidKi0;
double leftSpeedPidKd1 = leftSpeedPidKd0;
double leftSpeedPidKp2 = leftSpeedPidKp1;
double leftSpeedPidKi2 = leftSpeedPidKi1;
double leftSpeedPidKd2 = leftSpeedPidKd1;

double leftSpeedPidInput = 0;
int leftSpeedPidInputLast = 0; // debug
double leftSpeedPidOutput; // Power supplied to the motor PWM value.
double leftSpeedPidSetPoint;
PID leftSpeedPid(&leftSpeedPidInput, &leftSpeedPidOutput, &leftSpeedPidSetPoint, leftSpeedPidKp0, leftSpeedPidKi0, leftSpeedPidKd0, DIRECT); 
bool leftSpeedPidResult = false;

// right speed PID control
#define RIGHT_SPEED_PID_MIN_OUTPUT 15
#define RIGHT_SPEED_PID_MAX_OUTPUT 100
double rightSpeedPidKp0 = 10;
double rightSpeedPidKi0 = 3;
double rightSpeedPidKd0 = 0;

double rightSpeedPidKp1 = rightSpeedPidKp0;
double rightSpeedPidKi1 = rightSpeedPidKi0;
double rightSpeedPidKd1 = rightSpeedPidKd0;
double rightSpeedPidKp2 = rightSpeedPidKp1;
double rightSpeedPidKi2 = rightSpeedPidKi1;
double rightSpeedPidKd2 = rightSpeedPidKd1;

double rightSpeedPidInput = 0;
int rightSpeedPidInputLast = 0; // debug
double rightSpeedPidOutput; // Power supplied to the motor PWM value.
double rightSpeedPidSetPoint;
PID rightSpeedPid(&rightSpeedPidInput, &rightSpeedPidOutput, &rightSpeedPidSetPoint, rightSpeedPidKp0, rightSpeedPidKi0, rightSpeedPidKd0, DIRECT); 
bool rightSpeedPidResult = false;

// control variables
int msgFlag = 0;
double encoderPublisherTimer;

void elpin_isr(void) {
    uint8_t p1val = digitalRead(ELPIN1);
    uint8_t p2val = digitalRead(ELPIN2);
    uint8_t s = encoderLeftState & 3;
    if (p1val) s |= 4;
    if (p2val) s |= 8;
    encoderLeftState = (s >> 2);

    switch (s) {
        case 1: case 7: case 8: case 14:
            encoderLeftPulses++;
	    encoderLeftPulsesSpeedPID++;
	    encoderLeftPulsesSteeringPID++;
            return;
        case 2: case 4: case 11: case 13:
            encoderLeftPulses--;
	    encoderLeftPulsesSpeedPID--;
	    encoderLeftPulsesSteeringPID--;
            return;
        case 3: case 12:
            encoderLeftPulses += 2;
	    encoderLeftPulsesSpeedPID += 2;
	    encoderLeftPulsesSteeringPID += 2;
            return;
        case 6: case 9:
            encoderLeftPulses -= 2;
	    encoderLeftPulsesSpeedPID -= 2;
	    encoderLeftPulsesSteeringPID -= 2;
            return;
    }
}

void erpin_isr(void) {
    uint8_t p1val = digitalRead(ERPIN1);
    uint8_t p2val = digitalRead(ERPIN2);
    uint8_t s = encoderRightState & 3;
    if (p1val) s |= 4;
    if (p2val) s |= 8;
    encoderRightState = (s >> 2);

    switch (s) {
        case 1: case 7: case 8: case 14:
            encoderRightPulses--;
	    encoderRightPulsesSpeedPID--;
	    encoderRightPulsesSteeringPID--;
            return;
        case 2: case 4: case 11: case 13:
            encoderRightPulses++;
            encoderRightPulsesSpeedPID++;
            encoderRightPulsesSteeringPID++;
            return;
        case 3: case 12:
            encoderRightPulses -= 2;
            encoderRightPulsesSpeedPID -= 2;
            encoderRightPulsesSteeringPID -= 2;
            return;
        case 6: case 9:
            encoderRightPulses += 2;
            encoderRightPulsesSpeedPID += 2;
            encoderRightPulsesSteeringPID += 2;
            return;
    }
}

void updateMotorLeft(){
  if (leftSpeedPidSetPointDirection > 0){
    digitalWrite(PININ1, HIGH);
    digitalWrite(PININ2, LOW);
    softPwmWrite(PINEN1, leftMotorPwmOut);
    ROS_INFO("motorL FW: %d", leftMotorPwmOut);
  }

  if (leftSpeedPidSetPointDirection < 0){
    digitalWrite(PININ1, LOW);
    digitalWrite(PININ2, HIGH);
    softPwmWrite(PINEN1, leftMotorPwmOut);
    ROS_INFO("motorL BW: %d", leftMotorPwmOut);
  }

  if (leftSpeedPidSetPointDirection == 0){
    digitalWrite(PININ1, LOW);
    digitalWrite(PININ2, LOW);
    softPwmWrite(PINEN1, leftMotorPwmOut);
    ROS_INFO("motorL ID: %d", leftMotorPwmOut);
  }
}

void updateMotorRight(){
  if (rightSpeedPidSetPointDirection > 0){
    digitalWrite(PININ3, LOW);
    digitalWrite(PININ4, HIGH);
    softPwmWrite(PINEN2, rightMotorPwmOut);
    ROS_INFO("motorR FW: %d", rightMotorPwmOut);
  }

  if (rightSpeedPidSetPointDirection < 0){
    digitalWrite(PININ3, HIGH);
    digitalWrite(PININ4, LOW);
    softPwmWrite(PINEN2, rightMotorPwmOut);
    ROS_INFO("motorR BW: %d", rightMotorPwmOut);
  }

  if (rightSpeedPidSetPointDirection == 0){
    digitalWrite(PININ3, LOW);
    digitalWrite(PININ4, LOW);
    softPwmWrite(PINEN2, rightMotorPwmOut);
    ROS_INFO("motorR ID: %d", rightMotorPwmOut);
  }
}

void twistCallback(const geometry_msgs::Twist& msg) {
    ROS_INFO("twistCallback: %f, %f", msg.linear.x, msg.angular.z);
    speed_lin = msg.linear.x;
    speed_ang = msg.angular.z;
    msgFlag = 1;
}

void setupPid() {
  // steering PID
  steeringPidSetPoint = STEERING_PID_TARGET;
  steeringPid.SetMode(MANUAL);
  steeringPid.SetSampleTime(STEERING_PID_SAMPLE_TIME);
  steeringPid.SetOutputLimits(STEERING_PID_MIN_OUTPUT, STEERING_PID_MAX_OUTPUT);

  // left speed PID
  leftSpeedPidSetPoint = SPEED_PID_TARGET;
  leftSpeedPid.SetMode(AUTOMATIC);
  leftSpeedPid.SetSampleTime(SPEED_PID_SAMPLE_TIME);
  leftSpeedPid.SetOutputLimits(LEFT_SPEED_PID_MIN_OUTPUT, LEFT_SPEED_PID_MAX_OUTPUT);

  // right speed PID
  rightSpeedPidSetPoint = SPEED_PID_TARGET; // speed in pulses per sample time
  rightSpeedPid.SetMode(AUTOMATIC);
  rightSpeedPid.SetSampleTime(SPEED_PID_SAMPLE_TIME);
  rightSpeedPid.SetOutputLimits(RIGHT_SPEED_PID_MIN_OUTPUT, RIGHT_SPEED_PID_MAX_OUTPUT);
}

bool leftSpeedPidCompute() {
  bool pidResult;
  // prepare pid input
  if(useSteeringPid) {
    leftSpeedPidInput = abs(encoderLeftPulsesSpeedPID) + (abs(encoderLeftPulsesSpeedPID) * steeringPidOutput / 100);
  } else {
    leftSpeedPidInput = abs(encoderLeftPulsesSpeedPID);
  }
  // adaptative speed PID
  if(abs(leftSpeedPidInput - leftSpeedPidSetPoint) > SPEED_PID_ADAPTATIVE_LIMIT2) {
    leftSpeedPid.SetTunings(leftSpeedPidKp2, leftSpeedPidKi2, leftSpeedPidKd2);
  } else if(abs(leftSpeedPidInput - leftSpeedPidSetPoint) > SPEED_PID_ADAPTATIVE_LIMIT1) {
    leftSpeedPid.SetTunings(leftSpeedPidKp1, leftSpeedPidKi1, leftSpeedPidKd1);
  } else {
    leftSpeedPid.SetTunings(leftSpeedPidKp0, leftSpeedPidKi0, leftSpeedPidKd0);
  }
  // calculate pid
  pidResult = leftSpeedPid.Compute();
  if(pidResult) {
    encoderLeftPulses = 0;
    leftSpeedPidInputLast = leftSpeedPidInput;
  }
  return pidResult;
}

bool rightSpeedPidCompute() {
  bool pidResult;
  // prepare pid input
  if(useSteeringPid) {
    rightSpeedPidInput = abs(encoderRightPulsesSpeedPID) - (abs(encoderRightPulsesSpeedPID) * steeringPidOutput / 100);
  } else {
    rightSpeedPidInput = abs(encoderRightPulsesSpeedPID);
  }
  // adaptative speed PID
  if(abs(rightSpeedPidInput - rightSpeedPidSetPoint) > SPEED_PID_ADAPTATIVE_LIMIT2) {
    rightSpeedPid.SetTunings(rightSpeedPidKp2, rightSpeedPidKi2, rightSpeedPidKd2);
  } else if(abs(rightSpeedPidInput - rightSpeedPidSetPoint) > SPEED_PID_ADAPTATIVE_LIMIT1) {
    rightSpeedPid.SetTunings(rightSpeedPidKp1, rightSpeedPidKi1, rightSpeedPidKd1);
  } else {
    rightSpeedPid.SetTunings(rightSpeedPidKp0, rightSpeedPidKi0, rightSpeedPidKd0);
  }
  // calculate pid
  pidResult = rightSpeedPid.Compute();
  if(pidResult) {
    encoderRightPulses = 0;
    rightSpeedPidInputLast = rightSpeedPidInput;
  }
  return pidResult;
}

bool steeringPidCompute() {
  // calculate pwm wheel diference correction using pid
  bool pidResult;
  // prepare pid input
  steeringPidInput = abs(encoderLeftPulsesSteeringPID) - abs(encoderRightPulsesSteeringPID);
  // adaptative steering PID
  if(!steeringPidAdaptativeStart) {
    if(steeringPidInput > STEERING_PID_ADAPTATIVE_LIMIT) {
      steeringPid.SetTunings(steeringPidKp1, steeringPidKi1, steeringPidKd1);
    } else {
      steeringPid.SetTunings(steeringPidKp0, steeringPidKi0, steeringPidKd0);
    }
  } else {
    if(millis() > steeringPidAdaptativeTimer) {
      steeringPidAdaptativeStart = true;
    }
  }
  // calculate pid
  pidResult = steeringPid.Compute();
  if(pidResult) {
    steeringPidInputLast = steeringPidInput;
  }
  return pidResult;
}

void updatePid() {
    // calculate steeringPid
    if(useSteeringPid) {
      steeringdPidResult = steeringPidCompute();
    } else steeringdPidResult = false;

    // calculate speedPid
    leftSpeedPidResult = leftSpeedPidCompute();
    rightSpeedPidResult = rightSpeedPidCompute();
    // set speed
    leftMotorPwmOut = leftSpeedPidOutput;
    rightMotorPwmOut = rightSpeedPidOutput;

    if(speed_lin == 0 && speed_ang == 0) {
      leftMotorPwmOut = 0;
      rightMotorPwmOut = 0;
    }
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

    setupPid();

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

	// update motors on new cmd_vel message
        if(msgFlag) {
	    // calculate motors speed

	    ROS_INFO("new vel: %f %f", speed_lin, speed_ang);

            leftMotorSpeedRequest = (speed_lin/wheel_per) - ((speed_ang*wheel_sep)/wheel_per);
            rightMotorSpeedRequest = (speed_lin/wheel_per) + ((speed_ang*wheel_sep)/wheel_per);

	    ROS_INFO("conv1 vel: %f %f", leftMotorSpeedRequest, rightMotorSpeedRequest);

	    // apply min and max velocity restrictions
	    if(abs(rightMotorSpeedRequest) == abs(leftMotorSpeedRequest) && rightMotorSpeedRequest != 0) {
	      // original velociy are the same
	      if(rightMotorSpeedRequest > maxPidVel) {
	        if(rightMotorSpeedRequest > 0) rightMotorSpeedRequest = maxPidVel; else rightMotorSpeedRequest = -maxPidVel;
	        if(leftMotorSpeedRequest > 0) leftMotorSpeedRequest = maxPidVel; else leftMotorSpeedRequest = -maxPidVel;
	      }
	      if(rightMotorSpeedRequest < minPidVel) {
	        if(rightMotorSpeedRequest > 0) rightMotorSpeedRequest = minPidVel; else rightMotorSpeedRequest = -minPidVel;
	        if(leftMotorSpeedRequest > 0) leftMotorSpeedRequest = minPidVel; else leftMotorSpeedRequest = -minPidVel;
	      }
	    } else if(abs(rightMotorSpeedRequest) > abs(leftMotorSpeedRequest)) {
	      // right velocity is greater
	      if(rightMotorSpeedRequest > maxPidVel) {
	        leftMotorSpeedRequest = leftMotorSpeedRequest * (maxPidVel / rightMotorSpeedRequest);
	        if(rightMotorSpeedRequest >= 0) rightMotorSpeedRequest = maxPidVel; else rightMotorSpeedRequest = -maxPidVel;
	      }
	      if(rightMotorSpeedRequest < minPidVel) {
	        leftMotorSpeedRequest = leftMotorSpeedRequest * (minPidVel / rightMotorSpeedRequest);
	        if(rightMotorSpeedRequest >= 0) rightMotorSpeedRequest = maxPidVel; else rightMotorSpeedRequest = -minPidVel;
	      }
	    } else if(abs(rightMotorSpeedRequest) < abs(leftMotorSpeedRequest)) {
	      // left velocity is greater
	      if(leftMotorSpeedRequest > maxPidVel) {
	        rightMotorSpeedRequest = rightMotorSpeedRequest * (maxPidVel / leftMotorSpeedRequest);
	        if(leftMotorSpeedRequest >= 0) leftMotorSpeedRequest = maxPidVel; else leftMotorSpeedRequest = -maxPidVel;
	      }
	      if(leftMotorSpeedRequest < minPidVel) {
	        rightMotorSpeedRequest = rightMotorSpeedRequest * (minPidVel / leftMotorSpeedRequest);
	        if(leftMotorSpeedRequest >= 0) leftMotorSpeedRequest = minPidVel; else leftMotorSpeedRequest = -minPidVel;
	      }
	    }

	    ROS_INFO("limited vel: %f %f", leftMotorSpeedRequest, rightMotorSpeedRequest);

	    // transform to encoder pulses
	    rightMotorSpeedRequest = rightMotorSpeedRequest * wheel_encoder_pulses / 1000 * SPEED_PID_SAMPLE_TIME;
	    leftMotorSpeedRequest = leftMotorSpeedRequest * wheel_encoder_pulses / 1000 * SPEED_PID_SAMPLE_TIME;

	    ROS_INFO("pulses vel: %f %f", leftMotorSpeedRequest, rightMotorSpeedRequest);

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

	    ROS_INFO("motor: %d %f %d %f", leftSpeedPidSetPointDirection, leftSpeedPidSetPoint, rightSpeedPidSetPointDirection, rightSpeedPidSetPoint);

	    // switch on/off the steering pid
	    if(leftSpeedPidSetPoint == rightSpeedPidSetPoint && leftSpeedPidSetPoint != 0) {
	      // rotate or linear translation
	      steeringPid.SetMode(AUTOMATIC);
	      useSteeringPid = true;
	      encoderLeftPulsesSteeringPID = encoderRightPulsesSteeringPID = 0;
	    } else {
	      // curve
	      steeringPid.SetMode(MANUAL);
	      useSteeringPid = false;
	      encoderLeftPulsesSteeringPID = encoderRightPulsesSteeringPID = 0;
	    }

	    ROS_INFO("steering pid: %d", useSteeringPid);
	    
	    // machine state update
	    if(leftSpeedPidSetPoint == 0 && rightSpeedPidSetPoint == 0) {
	      // idle
	      motion_idle_timer = millis() + MOTION_IDLE_DELAY;
	      baseMotionState = MOTION_STOP;
	    } else if(useSteeringPid) {
	    // rotate or linear translation
	      if(leftSpeedPidSetPointDirection == rightSpeedPidSetPointDirection) {
	        if(leftSpeedPidSetPointDirection > 0) {
		  // front linear translation
		  baseMotionState = MOTION_FORWARD;
	        } else {
		  // backwards linear translation
		  baseMotionState = MOTION_BACKWARD;
	        }
	      } else {
	        if(leftSpeedPidSetPointDirection < 0) {
		  // rotate left
		  baseMotionState = MOTION_ROTATE_LEFT;
	        } else {
		  //rotate_right
		  baseMotionState = MOTION_ROTATE_RIGHT;
	        }
	      }
	    } else {
	      if(leftSpeedPidSetPointDirection > 0) {
	        if(rightSpeedPidSetPoint > leftSpeedPidSetPoint) {
		  baseMotionState = MOTION_FORWARD_CURVE_LEFT;
	        } else {
		  baseMotionState = MOTION_FORWARD_CURVE_RIGHT;
	        }
	      } else {
	        if(rightSpeedPidSetPoint > leftSpeedPidSetPoint) {
		  baseMotionState = MOTION_BACKWARD_CURVE_LEFT;
	        } else {
		  baseMotionState = MOTION_BACKWARD_CURVE_RIGHT;
	        }
	      }
	    }

	    ROS_INFO("state: %d", baseMotionState);

            //motionStateMsg.data = baseMotionState;
	    //motionstate_rospub1.publish(&motionStateMsg);

	    msgFlag = 0;
        }

	updatePid();

        // change state after wait for wheels stop
        if(baseMotionState == MOTION_STOP && millis() >= motion_idle_timer) {
	  baseMotionState = MOTION_IDLE;
	  //motionStateMsg.data = baseMotionState;
	  //motionstate_rospub1.publish(&motionStateMsg);
	  motion_idle_timer = 0;
        }

        if(baseMotionState != MOTION_IDLE) {
	  if(baseMotionState == MOTION_STOP) {
	    leftMotorPwmOut = 0;
	    rightMotorPwmOut = 0;
	    updateMotorLeft();
	    updateMotorRight();
	  }
	  updateMotorLeft();
	  updateMotorRight();
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
