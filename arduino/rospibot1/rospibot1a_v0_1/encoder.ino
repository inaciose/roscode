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
    encoderRightPulses--;
    encoderRightPulsesSpeedPID--;
    encoderRightPulsesSteeringPID--;
  }
}
