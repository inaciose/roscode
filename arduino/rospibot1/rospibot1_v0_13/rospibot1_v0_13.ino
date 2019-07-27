// rospibot1 0.12
// done: debug messages on alternate serial (TTYS1 on bpi)
// update: battery sense (bugfix)

//#define SERIAL2_DEBUG

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
#include "pid.h"

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

// looptime control
unsigned long loopTimeLast;
unsigned long looptime;

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

  //rightSpeedPidSetPointTmp = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  //leftSpeedPidSetPointTmp = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  
  // calculate rotation per second on both motors (im not sure about angular velocity)
  rightSpeedPidSetPointTmp = (speed_lin/wheel_per) + ((speed_ang*wheel_sep)/wheel_per);
  leftSpeedPidSetPointTmp = (speed_lin/wheel_per) - ((speed_ang*wheel_sep)/wheel_per);
  //
  rightSpeedPidSetPointTmp = rightSpeedPidSetPointTmp * ENCODER_PULSES / 1000 * SPEED_PID_SAMPLE_TIME;
  leftSpeedPidSetPointTmp = leftSpeedPidSetPointTmp * ENCODER_PULSES / 1000 * SPEED_PID_SAMPLE_TIME;

  // set right motors direction and speed
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
  // set left motors direction and speed
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

}
ros::Subscriber<geometry_msgs::Twist> vel_rossub1("cmd_vel", &messageCb );

//
// main functions
//

void setup() {
  #ifdef SERIAL2_DEBUG
    Serial2.begin(115200);
    Serial2.println("rospibot start!");
  #endif

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
  Setup_PID();

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
  loopTimeLast = millis();
}

void loop() {

  // imu statuscheck
  if (!dmpReady) return;
  
  Update_PID();

  Motor_Left();
  Motor_Right();

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

  #ifdef SERIAL2_DEBUG
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

  encoderLeftDirection = true; //default -> Forward
  pinMode(ENCODER_LEFT_PINA,INPUT);
  pinMode(ENCODER_LEFT_PINB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PINB), ENCODER_LEFT_FUNCTION, ENCODER_LEFT_SIGNAL);

  encoderRightDirection = true; //default -> Forward  
  pinMode(ENCODER_RIGHT_PINA,INPUT);
  pinMode(ENCODER_RIGHT_PINB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PINB), ENCODER_RIGHT_FUNCTION, ENCODER_RIGHT_SIGNAL);

}

void Setup_Ultrasound() {
  //delay(50);
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

void Setup_PID() {
  // steering PID
  steeringPidSetPoint = STEERING_PID_TARGET; 
  steeringPid.SetMode(MANUAL); 
  //steeringPid.SetMode(AUTOMATIC); 
  steeringPid.SetSampleTime(STEERING_PID_SAMPLE_TIME); 
  steeringPid.SetOutputLimits(STEERING_PID_MIN_OUTPUT, STEERING_PID_MAX_OUTPUT);

  // left speed PID
  //leftSpeedPidSetPoint = SPEED_PID_TARGET;
  //leftSpeedPid.SetMode(MANUAL); 
  leftSpeedPidSetPoint = 0;
  leftSpeedPid.SetMode(AUTOMATIC); 
  leftSpeedPid.SetSampleTime(SPEED_PID_SAMPLE_TIME); 
  leftSpeedPid.SetOutputLimits(LEFT_SPEED_PID_MIN_OUTPUT, LEFT_SPEED_PID_MAX_OUTPUT);

  // right speed PID
  //rightSpeedPidSetPoint = SPEED_PID_TARGET; // speed in pulses per sample time
  //rightSpeedPid.SetMode(MANUAL); 
  rightSpeedPidSetPoint = 0; // speed in pulses per sample time
  rightSpeedPid.SetMode(AUTOMATIC); 
  rightSpeedPid.SetSampleTime(SPEED_PID_SAMPLE_TIME); 
  rightSpeedPid.SetOutputLimits(RIGHT_SPEED_PID_MIN_OUTPUT, RIGHT_SPEED_PID_MAX_OUTPUT);
}

//
// update functions
//

// do ros topic publish
void Update_Encoders() {
  if(millis() >= encoder_publisher_timer) {
    encoder_publisher_timer = millis() + ENCODERS_PUBLISHER_DELAY;
    // Left encoder ros topic publisher
    encoder_msg1.data = encoderLeftPulses;
    encoder_rospub1.publish(&encoder_msg1); 
    // Right encoder ros topic publisher
    encoder_msg2.data = encoderRightPulses;
    encoder_rospub2.publish(&encoder_msg2);
  }
}

// do ros topic publish
void Update_Ultrasound() {
  Process_Ultrasound();
  if(millis() >= ultrasound_publisher_timer) {
    ultrasound_publisher_timer = millis() + ULTRASOUND_PUBLISHER_DELAY;
    // ros topic publish
    ultrasound_msg1.data = sonarValue[0];
    ultrasound_rospub1.publish(&ultrasound_msg1);    
  } 
}

// do ros topic publish
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

// do ros topic publish
void Update_Batteries() {
  
  // do internal update at faster rate
  if(millis() >= battery_sense_timer) {
    battery_sense_timer = millis() + BATTERY_SENSE_DELAY;
    // do battery1
    batteryReadValue1 = analogRead(BATTERY_SENSE_PIN1);
    battery_volt1 = (battery_volt1 * BATTERY_OLD_RATE) + ((BATTERY_MAXVOLT1 * (batteryReadValue1 * BATTERY_MAXVOLTADC / BATTERY_MAXREAD1) / BATTERY_MAXVOLTADC) * BATTERY_NEW_RATE);
    // do battery2
    batteryReadValue2 = analogRead(BATTERY_SENSE_PIN2);
    battery_volt2 = (battery_volt2 * BATTERY_OLD_RATE) + ((BATTERY_MAXVOLT2 * (batteryReadValue2 * BATTERY_MAXVOLTADC / BATTERY_MAXREAD2) / BATTERY_MAXVOLTADC) * BATTERY_NEW_RATE);
    Serial2.println(battery_volt2);
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

// dont publish any topic
void Update_PID() {
  // calculate steeringPid    
  if(useSteeringPid) {
    steeringdPidResult = steeringPidCompute();
  } else steeringdPidResult = false;

  // calculate speedPid
  leftSpeedPidResult = leftSpeedPidCompute();
  rightSpeedPidResult = rightSpeedPidCompute();
  // set speed
  leftMotorPwmOut = leftSpeedPidOutput; // - steeringPidOutput;
  rightMotorPwmOut = rightSpeedPidOutput; // + steeringPidOutput;

  if(speed_lin == 0 && speed_ang == 0) {
    leftMotorPwmOut = 0;
    rightMotorPwmOut = 0;
  }  
}

//
// motor control functions
//

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

//
// encoders isr
//

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
    encoderLeftPulsesSpeedPID++;
    encoderLeftPulsesSteeringPID++;
  } else {
    encoderLeftPulses--;
    encoderLeftPulsesSpeedPID--;
    encoderLeftPulsesSteeringPID--;
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
    encoderRightPulsesSpeedPID++;
    encoderRightPulsesSteeringPID++;
  } else {
    encoderRightPulsesSpeedPID--;
    encoderRightPulsesSteeringPID--;
  }
}

//
// ultrasound functions
//

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

//
// PID functions
//

boolean leftSpeedPidCompute() {
  boolean pidResult;

  if(useSteeringPid) {
    //Serial.println("useSteeringPid LEFT");
    leftSpeedPidInput = abs(encoderLeftPulsesSpeedPID) + (abs(encoderLeftPulsesSpeedPID) * steeringPidOutput / 100);
  } else {
    leftSpeedPidInput = abs(encoderLeftPulsesSpeedPID);
  }

  if(abs(leftSpeedPidInput - leftSpeedPidSetPoint) > SPEED_PID_ADAPTATIVE_LIMIT2) {
    //Serial.println("agressive LEFT");
    leftSpeedPid.SetTunings(leftSpeedPidKp2, leftSpeedPidKi2, leftSpeedPidKd2);
  } else if(abs(leftSpeedPidInput - leftSpeedPidSetPoint) > SPEED_PID_ADAPTATIVE_LIMIT1) {
    //Serial.println("medium LEFT");
    leftSpeedPid.SetTunings(leftSpeedPidKp1, leftSpeedPidKi1, leftSpeedPidKd1);
  } else {
    //Serial.println("nice LEFT");
    leftSpeedPid.SetTunings(leftSpeedPidKp0, leftSpeedPidKi0, leftSpeedPidKd0);
  }
  
  pidResult = leftSpeedPid.Compute(); 
  if(pidResult) {
    encoderLeftPulsesSpeedPID = 0;
    leftSpeedPidInputLast = leftSpeedPidInput;
  }
  return pidResult;
}

boolean rightSpeedPidCompute() {
  boolean pidResult;

  if(useSteeringPid) {
    //Serial.println("useSteeringPid RIGHT");
    rightSpeedPidInput = abs(encoderRightPulsesSpeedPID) - (abs(encoderRightPulsesSpeedPID) * steeringPidOutput / 100); 
  } else {
    rightSpeedPidInput = abs(encoderRightPulsesSpeedPID);
  }  

  if(abs(rightSpeedPidInput - rightSpeedPidSetPoint) > SPEED_PID_ADAPTATIVE_LIMIT2) {
    //Serial.println("agressive RIGHT");
    rightSpeedPid.SetTunings(rightSpeedPidKp2, rightSpeedPidKi2, rightSpeedPidKd2);
  } else if(abs(rightSpeedPidInput - rightSpeedPidSetPoint) > SPEED_PID_ADAPTATIVE_LIMIT1) {  
    //Serial.println("medium RIGHT");
    rightSpeedPid.SetTunings(rightSpeedPidKp1, rightSpeedPidKi1, rightSpeedPidKd1);
  } else {
    //Serial.println("nice RIGHT");
    rightSpeedPid.SetTunings(rightSpeedPidKp0, rightSpeedPidKi0, rightSpeedPidKd0);
  }
  
  pidResult = rightSpeedPid.Compute(); 
  if(pidResult) {
    encoderRightPulsesSpeedPID = 0;
    //Serial.print("Compute RIGHT "); Serial.println(rightSpeedPidInput);
    rightSpeedPidInputLast = rightSpeedPidInput;
  }
  return pidResult;
}

boolean steeringPidCompute() {
  boolean pidResult;
  // calculate pwm wheel diference correction using pid
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
  pidResult = steeringPid.Compute();
  if(pidResult) {
    steeringPidInputLast = steeringPidInput;
  }
  return pidResult;
}
