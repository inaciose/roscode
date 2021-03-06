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
