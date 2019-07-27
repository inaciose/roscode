// rospibot1 0.19
// update: rosserial serial port change

#define ROSSERIAL_USE_SERIAL2

//#define USE_IMU

#define SERIAL_DEBUG
//#define SERIAL_DEBUG_SETUP
#define SERIAL_DEBUG_LOOP

/*
#ifdef SERIAL_DEBUG_SETUP || SERIAL_DEBUG_LOOP
  #ifndef SERIAL_DEBUG
    #define SERIAL_DEBUG
  #endif
#endif
*/

#ifdef SERIAL_DEBUG 
  #ifndef ROSSERIAL_USE_SERIAL2
    #define SERIAL_DEBUG2
  #endif
#endif

#ifdef ROSSERIAL_USE_SERIAL2
  #include <ArduinoHardwareAlt.h>
#else
  #include <ArduinoHardware.h>
#endif
#include <ros.h>
//#include "ros.h"
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
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
#ifdef USE_IMU
  #include "Wire.h"
#endif

// mpu6050 config
#ifdef USE_IMU
  #include "mpu.h"
#endif

// internal led config
#define PULSE_LED_PIN LED_BUILTIN
#define PULSE_LED_DELAY 1000
unsigned long pulseLedTimer;
boolean pulseLedStatus;

// looptime control
unsigned long loopTimeLast;
unsigned long loopTime;

// motion idle delay timer
#define MOTION_IDLE_DELAY 120
unsigned long motion_idle_timer;

// timeframe publisher timer
#define TF_PUBLISHER_DELAY 50
unsigned long tf_publisher_timer;

//
// ros publishers, subscribers and broadcasters
//
#include "ros_pub.h"
#include "ros_tf.h"

// motors control subscribers
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;

  //rightSpeedPidSetPointTmp = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  //leftSpeedPidSetPointTmp = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  
  // calculate rotation per second on both motors (im not sure about angular velocity)
  rightSpeedPidSetPointTmp = (speed_lin/wheel_per) + ((speed_ang*wheel_sep)/wheel_per);
  leftSpeedPidSetPointTmp = (speed_lin/wheel_per) - ((speed_ang*wheel_sep)/wheel_per);

  #ifdef SERIAL_DEBUG2
    Serial2.print("cmd_vel: ");
    Serial2.print(leftSpeedPidSetPointTmp); Serial2.print("\t");
    Serial2.print(rightSpeedPidSetPointTmp); Serial2.print("\t");
  #else
    Serial.print("cmd_vel: ");
    Serial.print(leftSpeedPidSetPointTmp); Serial.print("\t");
    Serial.print(rightSpeedPidSetPointTmp); Serial.print("\t");
  #endif  
 
  // apply min and max velocity restrictions
  if(abs(rightSpeedPidSetPointTmp) == abs(leftSpeedPidSetPointTmp) && rightSpeedPidSetPointTmp != 0) {
    // original velociy are the same
    if(rightSpeedPidSetPointTmp > maxPidVel) {
      if(rightSpeedPidSetPointTmp > 0) rightSpeedPidSetPointTmp = maxPidVel; else rightSpeedPidSetPointTmp = -maxPidVel;
      if(leftSpeedPidSetPointTmp > 0) leftSpeedPidSetPointTmp = maxPidVel; else leftSpeedPidSetPointTmp = -maxPidVel;
    }
    if(rightSpeedPidSetPointTmp < minPidVel) {
      if(rightSpeedPidSetPointTmp > 0) rightSpeedPidSetPointTmp = minPidVel; else rightSpeedPidSetPointTmp = -minPidVel;
      if(leftSpeedPidSetPointTmp > 0) leftSpeedPidSetPointTmp = minPidVel; else leftSpeedPidSetPointTmp = -minPidVel;      
    }
  } else if(abs(rightSpeedPidSetPointTmp) > abs(leftSpeedPidSetPointTmp)) {
    // right velocity is greater
    if(rightSpeedPidSetPointTmp > maxPidVel) {
      leftSpeedPidSetPointTmp = leftSpeedPidSetPointTmp * (maxPidVel / rightSpeedPidSetPointTmp);
      if(rightSpeedPidSetPointTmp >= 0) rightSpeedPidSetPointTmp = maxPidVel; else rightSpeedPidSetPointTmp = -maxPidVel;
    }   
    if(rightSpeedPidSetPointTmp < minPidVel) {
      leftSpeedPidSetPointTmp = leftSpeedPidSetPointTmp * (minPidVel / rightSpeedPidSetPointTmp);
      if(rightSpeedPidSetPointTmp >= 0) rightSpeedPidSetPointTmp = maxPidVel; else rightSpeedPidSetPointTmp = -minPidVel;
    }   
  } else if(abs(rightSpeedPidSetPointTmp) < abs(leftSpeedPidSetPointTmp)) {
    // left velocity is greater
    if(leftSpeedPidSetPointTmp > maxPidVel) {
      rightSpeedPidSetPointTmp = rightSpeedPidSetPointTmp * (maxPidVel / leftSpeedPidSetPointTmp);
      if(leftSpeedPidSetPointTmp >= 0) leftSpeedPidSetPointTmp = maxPidVel; else leftSpeedPidSetPointTmp = -maxPidVel;
    }   
    if(leftSpeedPidSetPointTmp < minPidVel) {
      rightSpeedPidSetPointTmp = rightSpeedPidSetPointTmp * (minPidVel / leftSpeedPidSetPointTmp);
      if(leftSpeedPidSetPointTmp >= 0) leftSpeedPidSetPointTmp = minPidVel; else leftSpeedPidSetPointTmp = -minPidVel;
    }
  }

  // transform to encoder pulses
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

  // machine state update
  if(leftSpeedPidSetPoint == 0 && rightSpeedPidSetPoint == 0) {
    // idle
    motion_idle_timer = millis() + MOTION_IDLE_DELAY;
    //baseMotionState = MOTION_IDLE;
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

  #ifdef SERIAL_DEBUG2
    Serial2.print(baseMotionState); Serial2.print("\t");
    Serial2.print(leftSpeedPidSetPointDirection); Serial2.print("\t");
    Serial2.print(rightSpeedPidSetPointDirection); Serial2.println("\t");
    Serial2.print(leftSpeedPidSetPoint); Serial2.print("\t");
    Serial2.print(rightSpeedPidSetPoint); Serial2.println("\t");
  #else
    Serial.print(baseMotionState); Serial.print("\t");
    Serial.print(leftSpeedPidSetPointDirection); Serial.print("\t");
    Serial.print(rightSpeedPidSetPointDirection); Serial.println("\t");
    Serial.print(leftSpeedPidSetPoint); Serial.print("\t");
    Serial.print(rightSpeedPidSetPoint); Serial.println("\t");
  #endif
}
ros::Subscriber<geometry_msgs::Twist> vel_rossub1("cmd_vel", &messageCb );

//
// main functions
//

void setup() {
  #ifdef SERIAL_DEBUG2
    Serial2.begin(115200);
    Serial2.println("rospibot start!");
  #else
    Serial.begin(115200);
    Serial.println("rospibot start!");
  #endif

  #ifdef USE_IMU
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif
    #ifdef SERIAL_DEBUG && SERIAL_DEBUG_SETUP
      #ifdef SERIAL_DEBUG2
        Serial2.println("i2c start!");
      #else      
        Serial.println("i2c start!");
      #endif
    #endif
  #endif
  
  // setup internal led to pulse
  pinMode(PULSE_LED_PIN, OUTPUT);
  digitalWrite(PULSE_LED_PIN, HIGH);
  pulseLedStatus = true;
  pulseLedTimer = millis() + PULSE_LED_DELAY;

  Setup_Motors();
  Setup_Encoders();
  Setup_Ultrasound();

  #ifdef SERIAL_DEBUG && SERIAL_DEBUG_SETUP
    #ifdef SERIAL_DEBUG2
      Serial2.println("sonar start!");
    #else
      Serial.println("sonar start!");
    #endif
  #endif

  #ifdef USE_IMU
    Setup_MPU6050();  
    #ifdef SERIAL_DEBUG && SERIAL_DEBUG_SETUP
      #ifdef SERIAL_DEBUG2
        Serial2.println("mpu start!");
      #else
        Serial.println("mpu start!");
      #endif
    #endif
  #endif
  
  Setup_Batteries();
  Setup_PID();

  // ros init and brodcaster init for tf
  nh.initNode();
  broadcaster.init(nh);
  
  // topic registration
  nh.subscribe(vel_rossub1);
  nh.advertise(encoder_rospub1);
  nh.advertise(encoder_rospub2);
  
  nh.advertise(usrange_rospub0);
  nh.advertise(usrange_rospub1);
  nh.advertise(usrange_rospub2);
  nh.advertise(usrange_rospub3);
  nh.advertise(usrange_rospub4);
  nh.advertise(usrange_rospub5);
  nh.advertise(usrange_rospub6);

  #ifdef USE_IMU
    nh.advertise(imu_rospub1);
    nh.advertise(imu_rospub2);   
    nh.advertise(imu_rospub3);
    #ifdef OUTPUT_READABLE_QUATERNION
      nh.advertise(imu_rospub4);
    #endif
  #endif
  
  nh.advertise(battery_rospub1);
  nh.advertise(battery_rospub2);

  nh.spinOnce();

  #ifdef SERIAL_DEBUG && SERIAL_DEBUG_SETUP
    #ifdef SERIAL_DEBUG2
      Serial2.println("rosserial done!");
    #else
      Serial.println("rosserial done!");
    #endif
  #endif

  // ros publisher timers
  encoder_publisher_timer = millis() + ENCODERS_PUBLISHER_DELAY;
  #ifdef USE_IMU
    mpu_publisher_timer = millis() + MPU_PUBLISHER_DELAY;
  #endif
  battery_publisher_timer = millis() + BATTERY_PUBLISHER_DELAY;
  tf_publisher_timer = millis() + TF_PUBLISHER_DELAY;

  // other timers
  battery_sense_timer = millis() + BATTERY_SENSE_DELAY;
  motion_idle_timer = 0;
  loopTimeLast = millis();

  #ifdef SERIAL_DEBUG && SERIAL_DEBUG_SETUP
    #ifdef SERIAL_DEBUG2
      Serial2.println("rospibot setup done!");
    #else
      Serial.println("rospibot setup done!");
    #endif
  #endif
}

void Update_Odometry() {
  static long encoderRPosPrev = 0;
  static long encoderLPosPrev = 0;
  
  float SR = distancePerPulse * (encoderRightPulses - encoderRPosPrev);
  float SL = distancePerPulse * (encoderLeftPulses - encoderLPosPrev);

  encoderRPosPrev = encoderRightPulses;
  encoderLPosPrev = encoderLeftPulses;
/*
  Serial2.print(encoderLeftPulses); Serial2.print("\t");
  Serial2.print(encoderRightPulses); Serial2.print("\t");
  Serial2.print(encoderLeftPulses - encoderLPosPrev); Serial2.print("\t");
  Serial2.print(encoderRightPulses - encoderRPosPrev); Serial2.print("\t");
  Serial2.print(SL); Serial2.print("\t");
  Serial2.print(SL); Serial2.print("\t");
*/

  switch (baseMotionState) {
    case MOTION_ROTATE_LEFT:
    case MOTION_ROTATE_RIGHT:
      break;
    default:
      bodyX += SR * cos(bodyTheta);           
      bodyY += SL * sin(bodyTheta);
      break;    
  }
    
  bodyTheta += (SR - SL) / wheel_sep;

  if(bodyTheta > 2*PI)
    bodyTheta -= 2*PI;
  else if(bodyTheta < -2*PI)
    bodyTheta += 2*PI;
/*
  Serial2.print(bodyX); Serial2.print("\t");
  Serial2.print(bodyY); Serial2.print("\t");
  Serial2.print(bodyTheta); Serial2.println("\t");
*/
}

void loop() {

  #ifdef USE_IMU
    // imu statuscheck
    if (!dmpReady) {
      #ifdef SERIAL_DEBUG
        #ifdef SERIAL_DEBUG2
          Serial2.println("ERROR dmp NOT READY!");
        #else
          Serial.println("ERROR dmp NOT READY!");
        #endif
      #endif    
      return;
    }
  #endif
  
  Update_PID();
  
  // change state after wait for wheels stop
  if(motion_idle_timer && millis() >= motion_idle_timer) {
    baseMotionState = MOTION_IDLE;
    motion_idle_timer = 0;
  }
  
  if(baseMotionState != MOTION_IDLE) {
    Motor_Left();
    Motor_Right();
  }

  Update_Encoders();

/*
  #ifdef SERIAL_DEBUG && SERIAL_DEBUG_LOOP
    #ifdef SERIAL_DEBUG2
      Serial2.println("Update_Encoders() done!");
    #else
      Serial.println("Update_Encoders() done!");
    #endif
  #endif    
*/

  Update_Ultrasound();

/*
  #ifdef SERIAL_DEBUG && SERIAL_DEBUG_LOOP
    #ifdef SERIAL_DEBUG2
      Serial2.println("Update_Ultrasound() done!");
    #else
      Serial.println("Update_Ultrasound() done!");
    #endif
  #endif    
*/

  #ifdef USE_IMU
    Update_MPU6050();

/*  
    #ifdef SERIAL_DEBUG && SERIAL_DEBUG_LOOP
      #ifdef SERIAL_DEBUG2
        Serial2.println("Update_MPU6050() done!");
      #else
        Serial.println("Update_MPU6050() done!");
      #endif
    #endif    
*/
  #endif


  Update_Batteries();

  // update transform at current time
  if(millis() >= tf_publisher_timer) {
    tf_publisher_timer = millis() + TF_PUBLISHER_DELAY;
    // do odometry update
    Update_Odometry();
    // tf prepare
    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    // set robot pose
    t.transform.translation.x = bodyX;
    t.transform.translation.y = bodyY;
    t.transform.rotation = tf::createQuaternionFromYaw(bodyTheta);

    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
  }
  
  nh.spinOnce();

/*
  #ifdef SERIAL_DEBUG && SERIAL_DEBUG_LOOP
    #ifdef SERIAL_DEBUG2
      Serial2.println("spinOnce() done!");
    #else
      Serial.println("spinOnce() done!");
    #endif
  #endif    
*/

  // pulse internal led to assure running state
  if(millis() >= pulseLedTimer)  {
    pulseLedTimer = millis() + PULSE_LED_DELAY;
    // do
    pulseLedStatus = !pulseLedStatus;
    digitalWrite(PULSE_LED_PIN, pulseLedStatus);    
  }

  #ifdef SERIAL_DEBUG
    //if(leftSpeedPidResult || rightSpeedPidResult) {
      loopTime = millis();
      /*
      static int loopTimeMax = 0;
      if(loopTime - loopTimeLast > loopTimeMax) {
        loopTimeMax = loopTime - loopTimeLast;
      }
      Serial2.print(loopTimeMax); Serial2.print("\t");
      */
      #ifdef SERIAL_DEBUG2
        serial2_loopdata(loopTime - loopTimeLast);
      #else
        serial_loopdata(loopTime - loopTimeLast);
      #endif
      
      //Serial2.println(loopTime - loopTimeLast);
      loopTimeLast = loopTime;
    //}
  #endif
}
