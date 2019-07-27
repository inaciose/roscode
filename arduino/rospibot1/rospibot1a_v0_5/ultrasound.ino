//
// ultrasound
//

void Setup_Ultrasound() {
  //delay(50);
  int i = 0, j;
  
  // init ros tf timestamps
  for(j=0; j < SONAR_NUM; j++) {
    sonarValueTs[j] = nh.now(); 
  }

  // init sonar readings
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
  
  // init sonar timer
  sonarPingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    sonarPingTimer[i] = sonarPingTimer[i - 1] + SONAR_PING_INTERVAL;

  // ros sensor_msg broadcast setup
  usrange_msg0.radiation_type = sensor_msgs::Range::ULTRASOUND;
  usrange_msg0.header.frame_id =  us0_frame;
  usrange_msg0.field_of_view = 0.26;  // fake?
  usrange_msg0.min_range = 0; //m?
  usrange_msg0.max_range = 2.0; //m?
  
  usrange_msg1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  usrange_msg1.header.frame_id =  us1_frame;
  usrange_msg1.field_of_view = 0.26;  // fake?
  usrange_msg1.min_range = 0; //m?
  usrange_msg1.max_range = 2.0; //m?

  usrange_msg2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  usrange_msg2.header.frame_id =  us2_frame;
  usrange_msg2.field_of_view = 0.26;  // fake?
  usrange_msg2.min_range = 0; //m?
  usrange_msg2.max_range = 2.0; //m?

  usrange_msg3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  usrange_msg3.header.frame_id =  us3_frame;
  usrange_msg3.field_of_view = 0.26;  // fake?
  usrange_msg3.min_range = 0; //m?
  usrange_msg3.max_range = 2.0; //m?

  usrange_msg4.radiation_type = sensor_msgs::Range::ULTRASOUND;
  usrange_msg4.header.frame_id =  us4_frame;
  usrange_msg4.field_of_view = 0.26;  // fake?
  usrange_msg4.min_range = 0; //m?
  usrange_msg4.max_range = 2.0; //m?

  usrange_msg5.radiation_type = sensor_msgs::Range::ULTRASOUND;
  usrange_msg5.header.frame_id =  us5_frame;
  usrange_msg5.field_of_view = 0.26;  // fake?
  usrange_msg5.min_range = 0; //m?
  usrange_msg5.max_range = 2.0; //m?

  usrange_msg6.radiation_type = sensor_msgs::Range::ULTRASOUND;
  usrange_msg6.header.frame_id =  us6_frame;
  usrange_msg6.field_of_view = 0.26;  // fake?
  usrange_msg6.min_range = 0; //m?
  usrange_msg6.max_range = 2.0; //m?
}

// do ros topic publish
void Update_Ultrasound() {
  Process_Ultrasound();
  if(sonarReadAvailable) {

    /*
    // set tf message for ultrasound frame
    us_t1.header.frame_id = base_link;
    switch(sonarIndexLast) {
      case 0: us_t1.child_frame_id = us0_frame; break;
      case 1: us_t1.child_frame_id = us1_frame; break;
      case 2: us_t1.child_frame_id = us2_frame; break;
      case 3: us_t1.child_frame_id = us3_frame; break;
      case 4: us_t1.child_frame_id = us4_frame; break;
      case 5: us_t1.child_frame_id = us5_frame; break;
      case 6: us_t1.child_frame_id = us6_frame; break;
    }
    
    us_t1.transform.translation.x = sonarPose[sonarIndexLast][0];
    us_t1.transform.translation.y = sonarPose[sonarIndexLast][1];
    
    us_t1.transform.rotation = tf::createQuaternionFromYaw(sonarPose[sonarIndexLast][2] * 0.0174532925); // degrees to radians
    us_t1.header.stamp = sonarValueTs[sonarIndexLast];
    
    broadcaster.sendTransform(us_t1);
    */

    // set sensor message
    switch(sonarIndexLast) {
      case 0:
        usrange_msg0.range = sonarValue[sonarIndexLast] / 100.0;
        //usrange_msg1.header.stamp = nh.now();
        usrange_msg0.header.stamp = sonarValueTs[sonarIndexLast];
        usrange_rospub0.publish(&usrange_msg0);
        break;
      case 1:
        usrange_msg1.range = sonarValue[sonarIndexLast] / 100.0;
        usrange_msg1.header.stamp = sonarValueTs[sonarIndexLast];
        usrange_rospub1.publish(&usrange_msg1);
        break;
      case 2:
        usrange_msg2.range = sonarValue[sonarIndexLast] / 100.0;
        usrange_msg2.header.stamp = sonarValueTs[sonarIndexLast];
        usrange_rospub2.publish(&usrange_msg2);
        break;
      case 3:
        usrange_msg3.range = sonarValue[sonarIndexLast] / 100.0;
        usrange_msg3.header.stamp = sonarValueTs[sonarIndexLast];
        usrange_rospub3.publish(&usrange_msg3);
        break;
      case 4:
        usrange_msg4.range = sonarValue[sonarIndexLast] / 100.0;
        usrange_msg4.header.stamp = sonarValueTs[sonarIndexLast];
        usrange_rospub4.publish(&usrange_msg4);
        break;
      case 5:
        usrange_msg5.range = sonarValue[sonarIndexLast] / 100.0;
        usrange_msg5.header.stamp = sonarValueTs[sonarIndexLast];
        usrange_rospub5.publish(&usrange_msg5);
        break;
      case 6:
        usrange_msg6.range = sonarValue[sonarIndexLast] / 100.0;
        usrange_msg6.header.stamp = sonarValueTs[sonarIndexLast];
        usrange_rospub6.publish(&usrange_msg6);
        break;

    }
    sonarReadAvailable = false;

/*
    #ifdef SERIAL_DEBUG
      #ifdef SERIAL_DEBUG2
        Serial2.print(sonarIndexLast); Serial2.print("\t");
        Serial2.print(us_t1.transform.translation.x); Serial2.print("\t");
        Serial2.print(us_t1.transform.translation.y); Serial2.print("\t");
        Serial2.print(sonarPose[sonarIndexLast][2] * 0.0174532925); Serial2.print("\t");
        Serial2.println(usrange_msg1.range);
      #else
        Serial.print(sonarIndexLast); Serial.print("\t");
        Serial.print(us_t1.transform.translation.x); Serial.print("\t");
        Serial.print(us_t1.transform.translation.y); Serial.print("\t");
        Serial.print(sonarPose[sonarIndexLast][2] * 0.0174532925); Serial.print("\t");
        Serial.println(usrange_msg1.range);
      #endif
    #endif
*/
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
    //sonarZeroCounter[sonarIndex] = 0;
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
    //sonarValue[sonarIndex] = sonarKalman[sonarIndex]; // with K filter
    sonarValue[sonarIndex] = sonarRead[sonarIndex]; // no filter
    // ros tf timestamp for sensor_msg
    sonarValueTs[sonarIndex] = nh.now();
    sonarIndexLast = sonarIndex;
    sonarReadAvailable = true;
  }
}

// Sensor ping cycle complete, do something with the results.
void UltrasoundOneSensorCycle() {
  // The following code would be replaced with your code that does something with the ping results.
  // do nothing... its displayed on main loop
  // must keep the function
}
