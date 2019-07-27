//
// added: ultrasound sensor_msg/Range publishers
//

#define SERIAL2_DEBUG

#include <PinChangeInterrupt.h>

#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>

ros::NodeHandle nh;

// general includes
#include "kalman.h"

// include device configs
#include "ultrasound.h"
#include "locomotion.h"
#include "pid.h"

// internal led config
#define PULSE_LED_PIN LED_BUILTIN
#define PULSE_LED_DELAY 1000
unsigned long pulseLedTimer;
boolean pulseLedStatus;

// looptime control
unsigned long loopTimeLast;
unsigned long looptime;

// motion idle delay timer
#define MOTION_IDLE_DELAY 120
unsigned long motion_idle_timer;

// just for debug
double speed_ang_tmp, speed_lin_tmp;

//
// ros publishers, subscribers and broadcasters
//
#include "ros_pub.h"
//#include "ros_tf.h"
/*
// ultrasound
geometry_msgs::TransformStamped us_t1;
char base_link[] = "/base_link";
//char odom[] = "/odom";
*/
char us0_frame[] = "/ultrasound0";
char us1_frame[] = "/ultrasound1";
char us2_frame[] = "/ultrasound2";
char us3_frame[] = "/ultrasound3";
char us4_frame[] = "/ultrasound4";
char us5_frame[] = "/ultrasound5";
char us6_frame[] = "/ultrasound6";

// subscriber callback to handle motion velocity target requests
void messageCb1( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;

  //w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  //w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  //rightSpeedPidSetPointTmp = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  //leftSpeedPidSetPointTmp = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  // calculate rotation per second on both motors (im not sure about angular velocity)
  rightSpeedPidSetPointTmp = (speed_lin/wheel_per) + ((speed_ang*wheel_sep)/wheel_per);
  leftSpeedPidSetPointTmp = (speed_lin/wheel_per) - ((speed_ang*wheel_sep)/wheel_per);

  #ifdef SERIAL_DEBUG21
    Serial2.print("cmd_vel: ");
    Serial2.print(leftSpeedPidSetPointTmp); Serial2.print("\t");
    Serial2.print(rightSpeedPidSetPointTmp); Serial2.print("\t");
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
    rightSpeedPidSetPoint = round(rightSpeedPidSetPointTmp);    
  } else if(rightSpeedPidSetPointTmp < 0) {
    rightSpeedPidSetPointDirection = -1;
    rightSpeedPidSetPoint = round(abs(rightSpeedPidSetPointTmp));
  } else {
    rightSpeedPidSetPointDirection = 0;
    rightSpeedPidSetPoint = 0;
  }
  
  // set left motors direction and speed
  if(leftSpeedPidSetPointTmp > 0) {
    leftSpeedPidSetPointDirection = 1;
    leftSpeedPidSetPoint = round(leftSpeedPidSetPointTmp);
  } else if(leftSpeedPidSetPointTmp < 0) {
    leftSpeedPidSetPointDirection = -1;
    leftSpeedPidSetPoint = round(abs(leftSpeedPidSetPointTmp));
  } else {
    leftSpeedPidSetPointDirection = 0;
    leftSpeedPidSetPoint = round(leftSpeedPidSetPointTmp);
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

  // 
  if(encoderPulsesTargetState == ENC_TARGET_READY) {
    // set left encoder target
    if(leftSpeedPidSetPointDirection >= 0) {
      encoderLeftPulsesTarget = encoderLeftPulses + (encoderLeftPulsesTarget * pulses_per_m);    
    } else {
      encoderLeftPulsesTarget = encoderLeftPulses - (encoderLeftPulsesTarget * pulses_per_m);          
    }
    // set right encoder target
    if(rightSpeedPidSetPointDirection >= 0) {
      encoderRightPulsesTarget = encoderRightPulses + (encoderRightPulsesTarget * pulses_per_m);    
    } else {
      encoderRightPulsesTarget = encoderRightPulses - (encoderRightPulsesTarget * pulses_per_m);          
    }
    // set control
    encoderLeftPulsesOnTarget = false;
    encoderRightPulsesOnTarget = false;
    encoderPulsesTargetState = ENC_TARGET_ENABLED;
  }

  /*
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
  */
  
  // just for debug
  speed_ang_tmp = speed_ang;
  speed_lin_tmp = speed_lin;
    
}

ros::Subscriber<geometry_msgs::Twist> vel_rossub1("cmd_vel", &messageCb1 );

// subscriber callback to handle motion distance target requests
void messageCb2( const std_msgs::Int32& msg){
  
  //encoderPulsesTargetState = ENC_TARGET_ENABLED;
  //encoderLeftPulsesTarget = encoderLeftPulses + (msg.data * pulses_per_m);
  //encoderRightPulsesTarget = encoderRightPulses + (msg.data * pulses_per_m);
  encoderPulsesTargetState = ENC_TARGET_READY;
  encoderLeftPulsesTarget = msg.data;
  encoderRightPulsesTarget = msg.data;
  
  Serial2.println(msg.data);
}
ros::Subscriber<std_msgs::Int32> dist_rossub1("cmd_dist", &messageCb2 );

void setup() {
  #ifdef SERIAL2_DEBUG
    Serial2.begin(115200);
    Serial2.println("rospibot pid test start!");
  #endif
  
  // setup internal led to pulse
  pinMode(PULSE_LED_PIN, OUTPUT);
  digitalWrite(PULSE_LED_PIN, HIGH);
  pulseLedStatus = true;
  pulseLedTimer = millis() + PULSE_LED_DELAY;

  // ros setup
  nh.initNode();
  nh.subscribe(vel_rossub1);
  nh.subscribe(dist_rossub1);
  
  nh.advertise(encoder_rospub1);
  nh.advertise(encoder_rospub2);
  
  nh.advertise(usrange_rospub0);
  nh.advertise(usrange_rospub1);
  nh.advertise(usrange_rospub2);
  nh.advertise(usrange_rospub3);
  nh.advertise(usrange_rospub4);
  nh.advertise(usrange_rospub5);
  nh.advertise(usrange_rospub6); 

  nh.spinOnce();

  // setup parts
  Setup_Motors();
  Setup_Encoders();
  Setup_PID();
  Setup_Ultrasound();
    
  // ros publisher timers
  encoder_publisher_timer = millis() + ENCODERS_PUBLISHER_DELAY;  
  
  // diag control setup
  loopTimeLast = millis();
}

void loop() {
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

  // check encoder targets
  if(encoderPulsesTargetState == ENC_TARGET_ENABLED) {
    
    //
    // check left encoder target
    //
    if(!encoderLeftPulsesOnTarget) {
      if(leftSpeedPidSetPointDirection >= 0) {
        if(encoderLeftPulses >= encoderLeftPulsesTarget) {
          encoderLeftPulsesOnTarget = true;  
        }
      } else {
        if(encoderLeftPulses < encoderLeftPulsesTarget) {
          encoderLeftPulsesOnTarget = true;  
        }      
      }
      // left stop on encoder target
      if(encoderLeftPulsesOnTarget) {
        Serial2.println("L ON TARGET");
        leftSpeedPidSetPoint = 0;
        leftSpeedPidSetPointDirection = 0; 
      }
    }
    
    //
    // check right encoder target
    //
    if(!encoderRightPulsesOnTarget) {
      if(rightSpeedPidSetPointDirection >= 0) {
        if(encoderRightPulses >= encoderRightPulsesTarget) {
          encoderRightPulsesOnTarget = true;  
        }
      } else {
        if(encoderRightPulses < encoderRightPulsesTarget) {
          encoderRightPulsesOnTarget = true;  
        }      
      }
      // left stop on encoder target
      if(encoderRightPulsesOnTarget) {
        Serial2.println("R ON TARGET");
        rightSpeedPidSetPoint = 0;
        rightSpeedPidSetPointDirection = 0;  
      }
    }
    
    //
    // encoders on target
    //
    if(encoderLeftPulsesOnTarget && encoderRightPulsesOnTarget) {
      Serial2.println("BOTH ON TARGET");
      encoderLeftPulsesOnTarget = false;
      encoderRightPulsesOnTarget = false;
      encoderPulsesTargetState = ENC_TARGET_DISABLE;
      //set timer for MOTION_IDLE state
      motion_idle_timer = millis() + MOTION_IDLE_DELAY; 
    }
  }

  Update_Ultrasound();

  nh.spinOnce();

  // pulse internal led to assure running state
  if(millis() >= pulseLedTimer)  {
    pulseLedTimer = millis() + PULSE_LED_DELAY;
    // do
    pulseLedStatus = !pulseLedStatus;
    digitalWrite(PULSE_LED_PIN, pulseLedStatus);    
  }
  
  looptime = millis();
  #ifdef SERIAL2_DEBUG
    
    if(leftSpeedPidResult || rightSpeedPidResult) {
      //Serial2.print(speed_lin_tmp); Serial2.print("\t");
      //Serial2.print(speed_ang_tmp); Serial2.print("\t");
  
      //Serial2.print(leftSpeedPidSetPointTmp); Serial2.print("\t");
      //Serial2.print(rightSpeedPidSetPointTmp); Serial2.print("\t");
      
      Serial2.print(baseMotionState); Serial2.print("\t");
      Serial2.print(useSteeringPid); Serial2.print("\t");
      
      Serial2.print(leftSpeedPidSetPoint); Serial2.print("\t");
      Serial2.print(rightSpeedPidSetPoint); Serial2.print("\t");
      
      Serial2.print(leftSpeedPidSetPointDirection); Serial2.print("\t");
      Serial2.print(rightSpeedPidSetPointDirection); Serial2.print("\t");
  
      Serial2.print(leftMotorPwmOut); Serial2.print("\t");
      Serial2.print(rightMotorPwmOut); Serial2.print("\t");
      if(leftSpeedPidResult) {
        Serial2.print(leftSpeedPidSetPoint - leftSpeedPidInputLast); Serial2.print("\t");
        leftSpeedPidResult = false;
      } else {
        Serial2.print("---\t");
      }

      if(rightSpeedPidResult) {
        Serial2.print(rightSpeedPidSetPoint - rightSpeedPidInputLast); Serial2.print("\t");
        rightSpeedPidResult = false;
      } else {
        Serial2.print("---\t");
      }

      Serial2.print(encoderLeftPulsesTarget); Serial2.print("\t");
      Serial2.print(encoderRightPulsesTarget); Serial2.print("\t");
  
      Serial2.print(encoderLeftPulses); Serial2.print("\t");
      Serial2.print(encoderRightPulses); Serial2.print("\t");
  
      Serial2.print(encoderLeftPulses - encoderRightPulses); Serial2.print("\t");
    
      //looptime = millis();
      Serial2.println(looptime - loopTimeLast);
      
    }
    loopTimeLast = looptime;
  #endif
}
