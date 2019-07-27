/*
# stop
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}' --once
# go front slow
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}' --once
*/

#define SERIAL2_DEBUG

#include <PinChangeInterrupt.h>

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

// motor bridge-h config
#define EN_L 6
#define IN1_L 51
#define IN2_L 50
 
#define EN_R 7
#define IN1_R 52
#define IN2_R 53

// Encoder LEFT pins definition
#define ENCODER_LEFT_FUNCTION encoderLeftCounter
#define ENCODER_LEFT_PINB 2 //B pin -> the interrupt pin 2/3
#define ENCODER_LEFT_PINA 22 //A pin -> the digital pin 9/10
#define ENCODER_LEFT_SIGNAL CHANGE
byte encoderLeftPinLast; // control
volatile long encoderLeftPulses; // the number of pulses
volatile long encoderLeftPulsesPID; // the number of pulses for PID
volatile long encoderLeftPulsesSteeringPID; // the number of pulses for PID, must be reset at same time
boolean encoderLeftDirection; // the rotation direction 

// Encoder RIGHT pins definition
#define ENCODER_RIGHT_FUNCTION encoderRightCounter
#define ENCODER_RIGHT_PINB 3 //B pin -> the interrupt pin 2/3
#define ENCODER_RIGHT_PINA 23 //A pin -> the digital pin 9/10
#define ENCODER_RIGHT_SIGNAL CHANGE
byte encoderRightPinLast; // control
volatile long encoderRightPulses; // the number of pulses
volatile long encoderRightPulsesPID; // the number of pulses PID
volatile long encoderRightPulsesSteeringPID; // the number of pulses for PID, must be reset at same time
boolean encoderRightDirection; // the rotation direction
 
// kinematics config
// wheel_rad is the wheel radius 
// wheel_sep is
// units are m, m/s, radian/s
#define ENCODER_PULSES 1920.0
//double w_r=0, w_l=0;
double wheel_rad = 0.034, wheel_sep = 0.200;
double speed_ang = 0, speed_lin = 0;
double wheel_per = 2 * PI * wheel_rad;
//double encoder_pulse_distance = wheel_per / ENCODER_PULSES;

int leftMotorPwmOut = 0;
int rightMotorPwmOut = 0;

// added for twist messages
double leftSpeedPidSetPointTmp;
int leftSpeedPidSetPointDirection;
double rightSpeedPidSetPointTmp;
int rightSpeedPidSetPointDirection;

#include "pid.h"

// looptime control
unsigned long loopTimeLast;
unsigned long looptime;

ros::NodeHandle nh;

std_msgs::Int32 int_encoder_msg1;
ros::Publisher encoder_data1("encl", &int_encoder_msg1);

std_msgs::Int32 int_encoder_msg2;
ros::Publisher encoder_data2("encr", &int_encoder_msg2);

// just for debug
double speed_ang_tmp, speed_lin_tmp;
//bool msgReceived = false;

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;

  //w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  //w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  //rightSpeedPidSetPointTmp = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  //leftSpeedPidSetPointTmp = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  
  //encoder_pulse_distance
  rightSpeedPidSetPointTmp = (speed_lin/wheel_per) + ((speed_ang*wheel_sep)/wheel_per);
  leftSpeedPidSetPointTmp = (speed_lin/wheel_per) - ((speed_ang*wheel_sep)/wheel_per);

  rightSpeedPidSetPointTmp = rightSpeedPidSetPointTmp * ENCODER_PULSES / 1000 * SPEED_PID_SAMPLE_TIME;
  leftSpeedPidSetPointTmp = leftSpeedPidSetPointTmp * ENCODER_PULSES / 1000 * SPEED_PID_SAMPLE_TIME;

  if(rightSpeedPidSetPointTmp > 0) {
    rightSpeedPidSetPointDirection = 1;
    rightSpeedPidSetPoint = rightSpeedPidSetPointTmp;
  } else if(rightSpeedPidSetPointTmp < 0) {
    rightSpeedPidSetPointDirection = -1;
    rightSpeedPidSetPoint = abs(rightSpeedPidSetPointTmp);
  } else {
    rightSpeedPidSetPointDirection = 1;
    rightSpeedPidSetPoint = rightSpeedPidSetPointTmp;
  }

  if(leftSpeedPidSetPointTmp > 0) {
    leftSpeedPidSetPointDirection = 1;
    leftSpeedPidSetPoint = leftSpeedPidSetPointTmp;
  } else if(leftSpeedPidSetPointTmp < 0) {
    leftSpeedPidSetPointDirection = -1;
    leftSpeedPidSetPoint = abs(leftSpeedPidSetPointTmp);
  } else {
    leftSpeedPidSetPointDirection = 0;
    leftSpeedPidSetPoint = leftSpeedPidSetPointTmp;
  }
  
  // just for debug
  speed_ang_tmp = speed_ang;
  speed_lin_tmp = speed_lin;
    
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
 
void setup(){
  
  #ifdef SERIAL2_DEBUG
    Serial2.begin(115200);
    Serial2.println("rospibot start!");
  #endif

  // setup parts
  Setup_Motors();
  Setup_Encoders();
  Setup_PID();

  // ros setup
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_data1);
  nh.advertise(encoder_data2);
  
  // diag control setup
  loopTimeLast = millis();
}

void loop(){
  
  Update_PID();

  Motor_Left();
  Motor_Right();

  Update_Encoders();
  
  nh.spinOnce();

  #ifdef SERIAL2_DEBUG
    
    //Serial2.print(speed_lin_tmp); Serial2.print("\t");
    //Serial2.print(speed_ang_tmp); Serial2.print("\t");

    //Serial2.print(leftSpeedPidSetPointTmp); Serial2.print("\t");
    //Serial2.print(rightSpeedPidSetPointTmp); Serial2.print("\t");
    
    Serial2.print(leftSpeedPidSetPoint); Serial2.print("\t");
    Serial2.print(rightSpeedPidSetPoint); Serial2.print("\t");
    
    Serial2.print(leftSpeedPidSetPointDirection); Serial2.print("\t");
    Serial2.print(rightSpeedPidSetPointDirection); Serial2.print("\t");

    Serial2.print(leftMotorPwmOut); Serial2.print("\t");
    Serial2.print(rightMotorPwmOut); Serial2.print("\t");
    
    Serial2.print(leftSpeedPidInputLast); Serial2.print("\t");
    Serial2.print(rightSpeedPidInputLast); Serial2.print("\t");

    Serial2.print(encoderLeftPulses); Serial2.print("\t");
    Serial2.print(encoderRightPulses); Serial2.print("\t");

    Serial2.print(encoderLeftPulses - encoderRightPulses); Serial2.print("\t");
    
    looptime = millis();
    Serial2.println(looptime - loopTimeLast);
    loopTimeLast = looptime;
  #endif
}

void Setup_Motors(){
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  digitalWrite(EN_L, LOW);
  digitalWrite(EN_R, LOW);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
}
 
void Setup_Encoders() {

  encoderLeftDirection = true; //default -> Forward
  pinMode(ENCODER_LEFT_PINA,INPUT);
  pinMode(ENCODER_LEFT_PINB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PINB), ENCODER_LEFT_FUNCTION, ENCODER_LEFT_SIGNAL);

  encoderRightDirection = true; //default -> Forward  
  pinMode(ENCODER_RIGHT_PINA,INPUT);
  pinMode(ENCODER_RIGHT_PINB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PINB), ENCODER_RIGHT_FUNCTION, ENCODER_RIGHT_SIGNAL);
 
}

// ros topic publish
void Update_Encoders() {
  int_encoder_msg1.data = encoderLeftPulses;
  encoder_data1.publish(&int_encoder_msg1); 

  int_encoder_msg2.data = encoderRightPulses;
  encoder_data2.publish(&int_encoder_msg2);  
}

void encoderLeftCounter() {
  int Lstate = digitalRead(ENCODER_LEFT_PINB);
  if((encoderLeftPinLast == LOW) && Lstate==HIGH) {
    int val = digitalRead(ENCODER_LEFT_PINA);
    if(val == LOW && encoderLeftDirection) {
      encoderLeftDirection = false; //Reverse
    } else if(val == HIGH && !encoderLeftDirection) {
      encoderLeftDirection = true;  //Forward
    }
  }
  encoderLeftPinLast = Lstate; 
  // true for increment front
  if(encoderLeftDirection) {
    encoderLeftPulses++;
    encoderLeftPulsesPID++;
  } else {
    encoderLeftPulses--;
    encoderLeftPulsesPID--;
  }
}

void encoderRightCounter() {
  int Lstate = digitalRead(ENCODER_RIGHT_PINB);
  if((encoderRightPinLast == LOW) && Lstate==HIGH) {
    int val = digitalRead(ENCODER_RIGHT_PINA);
    if(val == LOW && encoderRightDirection) {
      encoderRightDirection = false; //Reverse
    } else if(val == HIGH && !encoderRightDirection) {
      encoderRightDirection = true;  //Forward
    }
  }
  encoderRightPinLast = Lstate; 
  // false for increment front
  if(!encoderRightDirection) {
    encoderRightPulses++;
    encoderRightPulsesPID++;
  } else {
    encoderRightPulses--;
    encoderRightPulsesPID--;
  }
}

void Motor_Left(){
  if (leftSpeedPidSetPointDirection > 0){
    
    analogWrite(EN_L, leftMotorPwmOut);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }

  if (leftSpeedPidSetPointDirection < 0){
    analogWrite(EN_L, leftMotorPwmOut);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  }

  if (leftSpeedPidSetPointDirection == 0){
    analogWrite(EN_L, leftMotorPwmOut);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
} 
 
void Motor_Right(){
  if (rightSpeedPidSetPointDirection > 0){
    analogWrite(EN_R, rightMotorPwmOut);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }
  
  if (rightSpeedPidSetPointDirection < 0){
    analogWrite(EN_R, rightMotorPwmOut);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }
  
  if (rightSpeedPidSetPointDirection == 0){
    analogWrite(EN_R, rightMotorPwmOut);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }

}
