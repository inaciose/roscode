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

// Encoder pins definition
// Left encoder
#define Left_Encoder_PinA 2
#define Left_Encoder_PinB 22
volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;

//Right Encoder
#define Right_Encoder_PinA 3
#define Right_Encoder_PinB 23
volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;
 
// kinematics config
// wheel_rad is the wheel radius 
// wheel_sep is
// units are m, m/s, radian/s
double w_r=0, w_l=0;
double wheel_rad = 0.0325, wheel_sep = 0.295;
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
  Motors_init();
  SetupEncoders();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_data1);
  nh.advertise(encoder_data2);
  // control
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
    Serial2.print(Left_Encoder_Ticks); Serial2.print("\t");
    Serial2.print(Right_Encoder_Ticks); Serial2.print("\t");
    
    Serial2.println(looptime - loopTimeLast);
    loopTimeLast = looptime;
  #endif
}
 
 
void Motors_init(){
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
 
void SetupEncoders() {
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT);      // sets pin A as input  
  pinMode(Left_Encoder_PinB, INPUT);      // sets pin B as input
  //Attaching interrupt in Left_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);
  
  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT);      // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT);      // sets pin B as input
  //Attaching interrupt in Right_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING); 
}

//Update both encoder value through serial port
void Update_Encoders() {
  int_encoder_msg1.data = Left_Encoder_Ticks;
  encoder_data1.publish(&int_encoder_msg1); 

  int_encoder_msg2.data = Right_Encoder_Ticks;
  encoder_data2.publish(&int_encoder_msg2);  
}

//do_Left_Encoder() Definitions
void do_Left_Encoder() {
   // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Right_Encoder() Definitions

void do_Right_Encoder() {
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;  
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
