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
