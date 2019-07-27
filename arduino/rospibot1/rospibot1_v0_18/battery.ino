void Setup_Batteries() {
  pinMode(BATTERY_SENSE_PIN1, INPUT);
  pinMode(BATTERY_SENSE_PIN2, INPUT);
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
    //Serial2.println(battery_volt2);
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
