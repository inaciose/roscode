#ifdef SERIAL_DEBUG2

  void serial2_loopdata(unsigned long lt) {
  #ifdef SERIAL_DEBUG && SERIAL_DEBUG_LOOP
    //Serial2.print(leftSpeedPidSetPoint); Serial2.print("\t");
    //Serial2.print(rightSpeedPidSetPoint); Serial2.print("\t");
    
    //Serial2.print(leftSpeedPidSetPointDirection); Serial2.print("\t");
    //Serial2.print(rightSpeedPidSetPointDirection); Serial2.print("\t");
  
    //Serial2.print(leftMotorPwmOut); Serial2.print("\t");
    //Serial2.print(rightMotorPwmOut); Serial2.print("\t");
  
    /*
    if(leftSpeedPidResult) {
      leftSpeedPidResult= false;
      Serial2.print(leftSpeedPidSetPoint-leftSpeedPidInputLast); Serial2.print("\t");
    } else {
      rightSpeedPidResult = false;
      Serial2.print(rightSpeedPidSetPoint-rightSpeedPidInputLast); Serial2.print("\t");
    }
    */
  
    Serial2.print(encoderLeftPulses); Serial2.print("\t");
    Serial2.print(encoderRightPulses); Serial2.print("\t");
  
    Serial2.print(encoderLeftPulses - encoderRightPulses); Serial2.print("\t");
    Serial2.print(baseMotionState); Serial2.print("\t");
    
    Serial2.print(bodyX); Serial2.print("\t");
    Serial2.print(bodyY); Serial2.print("\t");
    Serial2.print(bodyTheta * 180 / PI); Serial2.print("\t"); // * 57.2957795
  
    Serial2.println(lt);
  #endif
  }

#else

  void serial_loopdata(unsigned long lt) {
    #ifdef SERIAL_DEBUG && SERIAL_DEBUG_LOOP
      Serial.print("LD: ");
      Serial.print(leftSpeedPidSetPoint); Serial.print("\t");
      Serial.print(rightSpeedPidSetPoint); Serial.print("\t");
      
      //Serial.print(leftSpeedPidSetPointDirection); Serial.print("\t");
      //Serial.print(rightSpeedPidSetPointDirection); Serial.print("\t");
    
      Serial.print(leftMotorPwmOut); Serial.print("\t");
      Serial.print(rightMotorPwmOut); Serial.print("\t");
    
      
      if(leftSpeedPidResult) {
        leftSpeedPidResult = false;
        Serial.print(leftSpeedPidSetPoint-leftSpeedPidInputLast); Serial.print("\t");
      } else {
        rightSpeedPidResult = false;
        Serial.print(rightSpeedPidSetPoint-rightSpeedPidInputLast); Serial.print("\t");
      }
      
    
      Serial.print(encoderLeftPulses); Serial.print("\t");
      Serial.print(encoderRightPulses); Serial.print("\t");
    
      Serial.print(encoderLeftPulses - encoderRightPulses); Serial.print("\t");
      Serial.print(baseMotionState); Serial.print("\t");
      
      Serial.print(bodyX); Serial.print("\t");
      Serial.print(bodyY); Serial.print("\t");
      Serial.print(bodyTheta * 180 / PI); Serial.print("\t"); // * 57.2957795
    
      Serial.println(lt);
    #endif
    }
  
#endif
