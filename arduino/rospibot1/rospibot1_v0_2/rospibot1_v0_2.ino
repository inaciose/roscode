// rospibot 0.2
// done: ultrasound 

// ultrasound config
#define trigPin 24
#define echoPin 25

// ultrasounds publisher timer
#define ULTRASOUND_PUBLISHER_DELAY 30
unsigned long ultrasound_publisher_timer;

// motor control config
#define EN_L 6
#define IN1_L 51
#define IN2_L 50
 
#define EN_R 7
#define IN1_R 52
#define IN2_R 53

// wheel encoders config
// Left encoder
#define Left_Encoder_PinA 2
#define Left_Encoder_PinB 22
volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;
// Right Encoder
#define Right_Encoder_PinA 3
#define Right_Encoder_PinB 23
volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;

// wheel encoders publisher timer
#define ENCODERS_PUBLISHER_DELAY 50
unsigned long encoder_publisher_timer;

// wheel speed
double w_r=0, w_l=0;
 
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0325, wheel_sep = 0.295;
//int lowSpeed = 200;
//int highSpeed = 50;
double speed_ang=0, speed_lin=0;
 
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// ultrasound publishers
std_msgs::Int16 int_ultrasound_msg1;
ros::Publisher ultrasound_data1("us1", &int_ultrasound_msg1);

// wheel encoders publishers
std_msgs::Int32 int_encoder_msg1;
ros::Publisher encoder_data1("encl", &int_encoder_msg1);

std_msgs::Int32 int_encoder_msg2;
ros::Publisher encoder_data2("encr", &int_encoder_msg2);

// motors control subscribers
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

// other declarations
void Setup_Motors();
void Setup_Encoders();
void Setup_Ultrasound(); 
void Motor_Left(int Pulse_Width1);
void Motor_Right(int Pulse_Width2);
 
void setup() {
  Setup_Motors();
  Setup_Encoders();
  Setup_Ultrasound();
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_data1);
  nh.advertise(encoder_data2);
  nh.advertise(ultrasound_data1); 
  
  encoder_publisher_timer = millis() + ENCODERS_PUBLISHER_DELAY;
  ultrasound_publisher_timer = millis() + ULTRASOUND_PUBLISHER_DELAY;
}

void loop() {
  Motor_Left(w_l*10);
  Motor_Right(w_r*10);

  Update_Encoders();
  Update_Ultrasound();
  
  nh.spinOnce();
}

void Setup_Motors() {
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

void Setup_Ultrasound() {
  pinMode(trigPin, OUTPUT); // marking pins to be read and write 
  pinMode(echoPin, INPUT);  
}

void Update_Encoders() {
  if(millis() >= encoder_publisher_timer) {
    encoder_publisher_timer = millis() + ENCODERS_PUBLISHER_DELAY;
    // L
    int_encoder_msg1.data = Left_Encoder_Ticks;
    encoder_data1.publish(&int_encoder_msg1); 
    // R
    int_encoder_msg2.data = Right_Encoder_Ticks;
    encoder_data2.publish(&int_encoder_msg2);
  }  
}

void Update_Ultrasound() {
  long duration;
  int distance;
  if(millis() >= ultrasound_publisher_timer) {
    ultrasound_publisher_timer = millis() + ULTRASOUND_PUBLISHER_DELAY;
  
    digitalWrite(trigPin, LOW); // This is just setup code 
    delayMicroseconds(2);       // So that ultrasound sensor would
    digitalWrite(trigPin, HIGH);// start reading
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW); 

    duration = pulseIn(echoPin, HIGH); 
    distance = (duration/2) / 29.1;

    if (distance >= 200 ) {
      distance = 0;
    }
    
    int_ultrasound_msg1.data = distance;
    ultrasound_data1.publish(&int_ultrasound_msg1);
    
  } 
}

void Motor_Left(int Pulse_Width1) {
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
 
void Motor_Right(int Pulse_Width2) {
 
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

void do_Left_Encoder() {
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
}

void do_Right_Encoder() {
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;  
}
