// rospibot1 0.10
// update: rewrite to multiple files

#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle nh;


// general includes
#include "Kalman.h"

// include device configs
#include "battery.h"
#include "ultrasound.h"
#include "locomotion.h"

//Make a pin into an interrupt
#include <PinChangeInterrupt.h>

//I2C Library
#include "Wire.h"

// mpu6050 config
#include "mpu.h"

// internal led config
#define PULSE_LED_PIN LED_BUILTIN
#define PULSE_LED_DELAY 1000
unsigned long pulseLedTimer;
boolean pulseLedStatus;

// timeframe publisher timer
#define TF_PUBLISHER_DELAY 50
unsigned long tf_publisher_timer;

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

// battery sensor publishers
std_msgs::Float32 battery_msg1;
ros::Publisher battery_rospub1("battery1", &battery_msg1);

std_msgs::Float32 battery_msg2;
ros::Publisher battery_rospub2("battery2", &battery_msg2);

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
void Setup_Batteries();
void Update_Encoders();
void Update_Ultrasound();
void Update_MPU6050();
void Update_Batteries();
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
  Setup_Batteries();

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
  nh.advertise(battery_rospub1);
  nh.advertise(battery_rospub2);

  // ros publisher timers
  encoder_publisher_timer = millis() + ENCODERS_PUBLISHER_DELAY;
  ultrasound_publisher_timer = millis() + ULTRASOUND_PUBLISHER_DELAY;
  mpu_publisher_timer = millis() + MPU_PUBLISHER_DELAY;
  battery_publisher_timer = millis() + BATTERY_PUBLISHER_DELAY;
  tf_publisher_timer = millis() + TF_PUBLISHER_DELAY;

  // other timers
  battery_sense_timer = millis() + BATTERY_SENSE_DELAY;
}

void loop() {

  // imu statuscheck
  if (!dmpReady) return;
  
  Motor_Left(w_l*10);
  Motor_Right(w_r*10);

  Update_Encoders();
  Update_Ultrasound();
  Update_MPU6050();
  Update_Batteries();

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
  /*
  pinMode(US_PIN_TRIGGER1, OUTPUT); // marking pins to be read and write 
  pinMode(US_PIN_ECHO1, INPUT);
  */

  delay(50);
  int i = 0, j;
  while(i < 7) {
    for(j=0; j < SONAR_NUM; j++) {
      sonarRead[j] = sonar[j].ping_cm() * SONAR_SCALE;
      if(sonarRead[j] != 0) {
        switch(j) {
          case 0: sonarValue[j] = sonarKalman0.getFilteredValue(sonarRead[j]); break;
          /*
          case 1: sonarValue[j] = sonarKalman1.getFilteredValue(sonarRead[j]); break;
          case 2: sonarValue[j] = sonarKalman2.getFilteredValue(sonarRead[j]); break;
          case 3: sonarValue[j] = sonarKalman3.getFilteredValue(sonarRead[j]); break;
          */
        }
      }
      delay(33);
    }
    i++;
  }

  sonarPingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    sonarPingTimer[i] = sonarPingTimer[i - 1] + SONAR_PING_INTERVAL;

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

void Setup_Batteries() {
  pinMode(BATTERY_SENSE_PIN1, INPUT);
  pinMode(BATTERY_SENSE_PIN2, INPUT);
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

  Process_Ultrasound();

  if(millis() >= ultrasound_publisher_timer) {
    ultrasound_publisher_timer = millis() + ULTRASOUND_PUBLISHER_DELAY;

    /*
    digitalWrite(US_PIN_TRIGGER1, LOW); 
    delayMicroseconds(2);
    digitalWrite(US_PIN_TRIGGER1, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_PIN_TRIGGER1, LOW); 

    ultrasound_duration = pulseIn(US_PIN_ECHO1, HIGH); 
    ultrasound_distance = (ultrasound_duration/2) / 29.1;

    if (ultrasound_distance >= US_MAX_DISTANCE1 ) {
      ultrasound_distance = 0;
    }
    */

    // ros topic publish
    //ultrasound_msg1.data = ultrasound_distance;
    ultrasound_msg1.data = sonarValue[0];
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

void Update_Batteries() {
  
  // do internal update at faster rate
  if(millis() >= battery_sense_timer) {
    battery_sense_timer = millis() + BATTERY_SENSE_DELAY;
    // do battery1
    batteryReadValue1 = analogRead(BATTERY_SENSE_PIN1);
    batteryReadValue1 = batteryReadValue1 * (BATTERY_MAXVOLT1 / BATTERY_MAXREAD1);
    battery_volt1 = (battery_volt1 * (BATTERY_OLD_RATE) + (float)batteryReadValue1 * (BATTERY_NEW_RATE)) / 2.0;
    // do battery2
    batteryReadValue2 = analogRead(BATTERY_SENSE_PIN2);
    batteryReadValue2 = batteryReadValue2 * (BATTERY_MAXVOLT2 / BATTERY_MAXREAD2);
    battery_volt2 = (battery_volt2 * (BATTERY_OLD_RATE) + (float)batteryReadValue2 * (BATTERY_NEW_RATE)) / 2.0;
  }
  
  // do ros topic publish at slower rate
  if(millis() >= battery_publisher_timer) {
    battery_publisher_timer = millis() + BATTERY_PUBLISHER_DELAY;
    // do battery1 ros topic publish
    battery_msg1.data = battery_volt1;
    battery_rospub1.publish(&battery_msg1);
    // do battery1 ros topic publish
    battery_msg2.data = battery_volt2;
    battery_rospub2.publish(&battery_msg2);
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

void Process_Ultrasound() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= sonarPingTimer[i]) {         // Is it this sensor's time to ping?
      sonarPingTimer[i] += SONAR_PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && sonarIndex == SONAR_NUM - 1) UltrasoundOneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[sonarIndex].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      sonarIndex = i;                          // Sensor being accessed.
      sonarRead[sonarIndex] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[sonarIndex].ping_timer(UltrasoundEchoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }  
}

void UltrasoundEchoCheck() {  
  // If ping received, set the sensor distance to array.
  if (sonar[sonarIndex].check_timer()) {
    sonarZeroCounter[sonarIndex] = 0;
    sonarRead[sonarIndex] = sonar[sonarIndex].ping_result / US_ROUNDTRIP_CM * SONAR_SCALE;
    
    // try to filter large discrepancies on current read compared with last read
    if(abs(sonarKalman[sonarIndex] - sonarRead[sonarIndex])  > sonarKalman[sonarIndex] * SONAR_MAX_ANOMALY / 100) {
      // there is an anomaly
      if(sonarAnomalyCounter[sonarIndex] < 1) { // HARDCODED
        // if bellow the limit use last stable value
        sonarRead[sonarIndex] = sonarKalman[sonarIndex];        
      }
      // increment counter to later trigger
      sonarAnomalyCounter[sonarIndex]++;
    } else {
      // there is no anomaly, reset counter
      sonarAnomalyCounter[sonarIndex] = 0;
    }

    // now after any previous adjusts to the input we calculate a stable value
    switch(sonarIndex) {
      case 0:
        sonarKalman[sonarIndex] = sonarKalman0.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      /*
      case 1:
        sonarKalman[sonarIndex] = sonarKalman1.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 2:
        sonarKalman[sonarIndex] = sonarKalman2.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 3:
        sonarKalman[sonarIndex] = sonarKalman3.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      */
    }
    // set current read value for stable
    sonarValue[sonarIndex] = sonarKalman[sonarIndex];
  } else {
    // we didnt receive any ping in sonar.check_timer() 
    // so we need to manage if the value is 0 or the stable reading
    if(sonarZeroCounter[sonarIndex] > SONAR_ZERO_MIN) {
      // we didnot receive a ping for enougth time to be zero
      sonarValue[sonarIndex] = 0;
    } else {
      // still wait we need to use last stable
      sonarValue[sonarIndex] = sonarKalman[sonarIndex];
    }
    // control the value 0
    sonarZeroCounter[sonarIndex]++;
    if(sonarZeroCounter[sonarIndex] > 65530) sonarZeroCounter[sonarIndex] = SONAR_ZERO_MIN + 1;
  }
}

// Sensor ping cycle complete, do something with the results.
void UltrasoundOneSensorCycle() {
  // The following code would be replaced with your code that does something with the ping results.
  // do nothing... its displayed on main loop
  // must keep the function
}
