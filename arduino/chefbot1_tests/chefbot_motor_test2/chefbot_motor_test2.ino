#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

/////////////////////////////////////////////////////////////////
//Motor Pin definition
//Left Motor pins
#define A_1 50
#define B_1 51
//PWM 1 pin number
#define PWM_1 6

//Right Motor
#define A_2 53
#define B_2 52
//PWM 2 pin number
#define PWM_2 7

///////////////////////////////////////////////////////////////////////////////////////
//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;

ros::NodeHandle  nh;

void motorLpwm_cb( const std_msgs::UInt16& cmd_msg){
  motor_left_speed = cmd_msg.data;
}

ros::Subscriber<std_msgs::UInt16> sub1("cmd_vel_l", motorLpwm_cb);

void motorRpwm_cb( const std_msgs::UInt16& cmd_msg){
  motor_right_speed = cmd_msg.data;
}

ros::Subscriber<std_msgs::UInt16> sub2("cmd_vel_r", motorRpwm_cb);


void setup(){
  pinMode(13, OUTPUT);

  SetupMotors();

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);

}

void loop() {
  nh.spinOnce();
  Update_Motors();
  delay(10);
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

}

void SetupMotors() {
  
  //Left motor
  pinMode(A_1,OUTPUT);
  pinMode(B_1,OUTPUT); 
  
  
  //Right Motor
  pinMode(A_2,OUTPUT);
  pinMode(B_2,OUTPUT);  
  
}

void Update_Motors() {
  
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);

}

void moveRightMotor(float rightServoValue) {
  if (rightServoValue>0) {
    digitalWrite(A_2,HIGH);
    digitalWrite(B_2,LOW);
    analogWrite(PWM_2,rightServoValue);
  } else if(rightServoValue<0) {
    digitalWrite(A_2,LOW);
    digitalWrite(B_2,HIGH);
    analogWrite(PWM_2,abs(rightServoValue)); 
  } else if(rightServoValue == 0) {
    digitalWrite(A_2,HIGH);
    digitalWrite(B_2,HIGH);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveLeftMotor(float leftServoValue) {
  if (leftServoValue > 0) {
    digitalWrite(A_1,LOW);
    digitalWrite(B_1,HIGH);
  analogWrite(PWM_1,leftServoValue);
  } else if(leftServoValue < 0) {
    digitalWrite(A_1,HIGH);
    digitalWrite(B_1,LOW);
  analogWrite(PWM_1,abs(leftServoValue));
  } else if(leftServoValue == 0) {
    digitalWrite(A_1,HIGH);
    digitalWrite(B_1,HIGH);
  }  
}
