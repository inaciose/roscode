
void serial2_loopdata(unsigned long lt) {
#ifdef SERIAL2_DEBUG && SERIAL2_DEBUG_LOOP
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
