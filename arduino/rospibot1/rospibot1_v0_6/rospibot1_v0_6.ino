// rospibot 0.6
// done: tf broadcaster

#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle nh;

// timeframe publisher timer
#define TF_PUBLISHER_DELAY 50
unsigned long tf_publisher_timer;

// internal led config
#define PULSE_LED_PIN LED_BUILTIN
#define PULSE_LED_DELAY 1000
unsigned long pulseLedTimer;
boolean pulseLedStatus;

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

//Make a pin into an interrupt
#include <PinChangeInterrupt.h>

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data 

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);
#define OUTPUT_READABLE_QUATERNION
#define MPU_INT_PIN 11
#define MPU_PUBLISHER_DELAY 20
unsigned long mpu_publisher_timer;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];

// ISR
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true;}

//
// publishers and subscribers
//

// ultrasound publishers
std_msgs::Int16 ultrasound_msg1;
ros::Publisher ultrasound_rospub1("us1", &ultrasound_msg1);

// wheel encoders publishers
std_msgs::Int32 encoder_msg1;
ros::Publisher encoder_rospub1("encl", &encoder_msg1);

std_msgs::Int32 encoder_msg2;
ros::Publisher encoder_rospub2("encr", &encoder_msg2);

// imu publishers
std_msgs::Float32 imu_msg1;
ros::Publisher imu_rospub1("imu1", &imu_msg1);

std_msgs::Float32 imu_msg2;
ros::Publisher imu_rospub2("imu2", &imu_msg2);

std_msgs::Float32 imu_msg3;
ros::Publisher imu_rospub3("imu3", &imu_msg3);

#ifdef OUTPUT_READABLE_QUATERNION
  std_msgs::Float32 imu_msg4;
  ros::Publisher imu_rospub4("imu4", &imu_msg4);
#endif

// transforms at current time publishers
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";

// motors control subscribers
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}
ros::Subscriber<geometry_msgs::Twist> vel_rossub1("cmd_vel", &messageCb );

// function declarations
void Setup_Motors();
void Setup_Encoders();
void Setup_Ultrasound();
void Setup_MPU6050();
void Update_Encoders();
void Update_Ultrasound();
void Update_MPU6050();
void Motor_Left(int Pulse_Width1);
void Motor_Right(int Pulse_Width2);
void do_Left_Encoder();
void do_Right_Encoder();

//
// main functions
//

void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  // setup internal led to pulse
  pinMode(PULSE_LED_PIN, OUTPUT);
  digitalWrite(PULSE_LED_PIN, HIGH);
  pulseLedStatus = true;
  pulseLedTimer = millis() + PULSE_LED_DELAY;
    
  Setup_Motors();
  Setup_Encoders();
  Setup_Ultrasound();
  Setup_MPU6050();

  // ros init and brodcaster init for tf
  nh.initNode();
  broadcaster.init(nh);
  
  // topic registration
  nh.subscribe(vel_rossub1);
  nh.advertise(encoder_rospub1);
  nh.advertise(encoder_rospub2);
  nh.advertise(ultrasound_rospub1);
  nh.advertise(imu_rospub1);
  nh.advertise(imu_rospub2);   
  nh.advertise(imu_rospub3);
  #ifdef OUTPUT_READABLE_QUATERNION
    nh.advertise(imu_rospub4);
  #endif

  // ros publisher timers
  encoder_publisher_timer = millis() + ENCODERS_PUBLISHER_DELAY;
  ultrasound_publisher_timer = millis() + ULTRASOUND_PUBLISHER_DELAY;
  mpu_publisher_timer = millis() + MPU_PUBLISHER_DELAY;
  tf_publisher_timer = millis() + TF_PUBLISHER_DELAY;
}

void loop() {

  // imu statuscheck
  if (!dmpReady) return;
  
  Motor_Left(w_l*10);
  Motor_Right(w_r*10);

  Update_Encoders();
  Update_Ultrasound();
  Update_MPU6050();

  // update transform at current time
  if(millis() >= tf_publisher_timer) {
    tf_publisher_timer = millis() + TF_PUBLISHER_DELAY;
    // do
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    t.transform.translation.x = 1.0; 
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0; 
    t.transform.rotation.z = 0.0; 
    t.transform.rotation.w = 1.0;  
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
  }
  
  nh.spinOnce();

  // pulse internal led to assure running state
  if(millis() >= pulseLedTimer)  {
    pulseLedTimer = millis() + PULSE_LED_DELAY;
    // do
    pulseLedStatus = !pulseLedStatus;
    digitalWrite(PULSE_LED_PIN, pulseLedStatus);    
  }
}

//
// setup functions
//

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

void Setup_MPU6050() {
  
  //Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  //Initialize DMP in MPU 6050
  //Setup_MPU6050_DMP();  
   //DMP Initialization
  
   devStatus = accelgyro.dmpInitialize();
   
   accelgyro.setXGyroOffset(220);
   accelgyro.setXGyroOffset(76);
   accelgyro.setXGyroOffset(-85); 
   accelgyro.setXGyroOffset(1788);  
  
  
   if(devStatus == 0){
    
    accelgyro.setDMPEnabled(true);
    
    pinMode(MPU_INT_PIN,INPUT_PULLUP);    
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    
    mpuIntStatus = accelgyro.getIntStatus();
    
    dmpReady = true;
    
    packetSize = accelgyro.dmpGetFIFOPacketSize();
     
   } else {
     ;
   }
  
    
}

//
// update functions
//

void Update_Encoders() {
  if(millis() >= encoder_publisher_timer) {
    encoder_publisher_timer = millis() + ENCODERS_PUBLISHER_DELAY;
    // Left encoder ros topic publisher
    encoder_msg1.data = Left_Encoder_Ticks;
    encoder_rospub1.publish(&encoder_msg1); 
    // Right encoder ros topic publisher
    encoder_msg2.data = Right_Encoder_Ticks;
    encoder_rospub2.publish(&encoder_msg2);
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

    // ros topic publish
    ultrasound_msg1.data = distance;
    ultrasound_rospub1.publish(&ultrasound_msg1);
    
  } 
}

void Update_MPU6050() {
  
  // using DMP processing data is ready?
  if (!mpuInterrupt) return;
    
  while (!mpuInterrupt && fifoCount < packetSize) {
      mpuInterrupt = true;
  }

  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();
  
  //get current FIFO count
  fifoCount = accelgyro.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount > 1024) {
      // reset so we can continue cleanly
      accelgyro.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_QUATERNION
      // get quaternion values in easy matrix form: w x y z
      accelgyro.dmpGetQuaternion(&q, fifoBuffer);

      if(millis() >= mpu_publisher_timer) {
        mpu_publisher_timer = millis() + MPU_PUBLISHER_DELAY;
      
        // ros topic publish
        imu_msg1.data = q.x;
        imu_rospub1.publish(&imu_msg1); 

        imu_msg2.data = q.y;
        imu_rospub2.publish(&imu_msg2); 
      
        imu_msg3.data = q.z;
        imu_rospub3.publish(&imu_msg3); 

        imu_msg4.data = q.w;
        imu_rospub4.publish(&imu_msg4);
      }               
    #endif

    #ifdef OUTPUT_READABLE_EULER
      // get Euler angles in degrees
      accelgyro.dmpGetQuaternion(&q, fifoBuffer);
      accelgyro.dmpGetEuler(euler, &q);
      
      if(millis() >= mpu_publisher_timer) {
        mpu_publisher_timer = millis() + MPU_PUBLISHER_DELAY;

        // ros topic publish
        imu_msg1.data = euler[0] * 180/M_PI;
        imu_rospub1.publish(&imu_msg1); 
        
        imu_msg2.data = euler[1] * 180/M_PI;
        imu_rospub2.publish(&imu_msg2); 

        imu_msg3.data = qeuler[2] * 180/M_PI;
        imu_rospub3.publish(&imu_msg3);
      }
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // get Euler angles in degrees
      accelgyro.dmpGetQuaternion(&q, fifoBuffer);
      accelgyro.dmpGetGravity(&gravity, &q);
      accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);

      if(millis() >= mpu_publisher_timer) {
        mpu_publisher_timer = millis() + MPU_PUBLISHER_DELAY;

        // ros topic publish
        imu_msg1.data = ypr[0] * 180/M_PI;
        imu_rospub1.publish(&imu_msg1); 
      
        imu_msg2.data = ypr[1] * 180/M_PI;
        imu_rospub2.publish(&imu_msg2); 

        imu_msg3.data = ypr[2] * 180/M_PI;
        imu_rospub3.publish(&imu_msg3);
      }      
    #endif
  }    
}

//
// other functions
//

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

// encoders isr
// Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
void do_Left_Encoder() {
  
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
}

void do_Right_Encoder() {
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;  
}
