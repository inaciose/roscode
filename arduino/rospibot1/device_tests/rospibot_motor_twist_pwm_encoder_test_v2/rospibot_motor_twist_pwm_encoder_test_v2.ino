// UPDATE TO USE SJ1 DEMO ENCODER CODE
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
boolean encoderLeftDirection; // the rotation direction 

// Encoder RIGHT pins definition
#define ENCODER_RIGHT_FUNCTION encoderRightCounter
#define ENCODER_RIGHT_PINB 3 //B pin -> the interrupt pin 2/3
#define ENCODER_RIGHT_PINA 23 //A pin -> the digital pin 9/10
#define ENCODER_RIGHT_SIGNAL CHANGE
byte encoderRightPinLast; // control
volatile long encoderRightPulses; // the number of pulses
volatile long encoderRightPulsesPID; // the number of pulses PID
boolean encoderRightDirection; // the rotation direction
 
// kinematics config
// wheel_rad is the wheel radius 
// wheel_sep is
// units are m, m/s, radian/s
double w_r=0, w_l=0;
double wheel_rad = 0.034, wheel_sep = 0.200;
double speed_ang=0, speed_lin=0;

// looptime control
unsigned long loopTimeLast;
unsigned long looptime;

ros::NodeHandle nh;

std_msgs::Int32 int_encoder_msg1;
ros::Publisher encoder_data1("encl", &int_encoder_msg1);

std_msgs::Int32 int_encoder_msg2;
ros::Publisher encoder_data2("encr", &int_encoder_msg2);

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);
 
void setup(){
  #ifdef SERIAL2_DEBUG
    Serial2.begin(115200);
    Serial2.println("rospibot start!");
  #endif
  
  Setup_Motors();
  Setup_Encoders();
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_data1);
  nh.advertise(encoder_data2);
  
  loopTimeLast = millis();
}
 
 
void loop(){
  MotorL(w_l*10);
  MotorR(w_r*10);
  Update_Encoders();
  
  nh.spinOnce();

  #ifdef SERIAL2_DEBUG
    looptime = millis();
    /*
    Serial2.print(w_l); Serial2.print("\t");
    Serial2.print(w_r); Serial2.print("\t");
    Serial2.print(w_l*10); Serial2.print("\t");
    Serial2.print(w_r*10); Serial2.print("\t");
    */
    Serial2.print(encoderLeftPulses); Serial2.print("\t");
    Serial2.print(encoderRightPulses); Serial2.print("\t");
    
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
  if(encoderLeftDirection) encoderLeftPulses++;
  else encoderLeftPulses--;
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
  if(!encoderRightDirection) encoderRightPulses++;
  else encoderRightPulses--;
}

void MotorL(int Pulse_Width1){
  if (Pulse_Width1 > 0){
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }

  if (Pulse_Width1 < 0){
    Pulse_Width1=abs(Pulse_Width1);
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  }

  if (Pulse_Width1 == 0){
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
} 
 
void MotorR(int Pulse_Width2){
  if (Pulse_Width2 > 0){
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }
  
  if (Pulse_Width2 < 0){
    Pulse_Width2=abs(Pulse_Width2);
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }
  
  if (Pulse_Width2 == 0){
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }

}
