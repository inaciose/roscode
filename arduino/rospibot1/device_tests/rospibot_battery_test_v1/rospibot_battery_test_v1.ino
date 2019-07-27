// battery sensor config
#define BATTERY_MAXVOLTADC 5.0
#define BATTERY_OLD_RATE 0.8
#define BATTERY_NEW_RATE 0.2
// bat1
#define BATTERY_SENSE_PIN1 A1
#define BATTERY_MAXREAD1 1024.0
#define BATTERY_MAXVOLT1 14.3 //(12.6V)
int batteryReadValue1 = 0;
float battery_volt1 = BATTERY_MAXVOLT1 / 2.0;
// bat2
#define BATTERY_SENSE_PIN2 A0
#define BATTERY_MAXREAD2 1024.0
#define BATTERY_MAXVOLT2 4.9 //(4.1V)
float battery_volt2 = BATTERY_MAXVOLT2 / 2.0;
int batteryReadValue2 = 0;

// battery sensors publisher timer
#define BATTERY_PUBLISHER_DELAY 5000
#define BATTERY_SENSE_DELAY 500
unsigned long battery_publisher_timer;
unsigned long battery_sense_timer;


void setup() {
  Serial.begin(115200);
  pinMode(BATTERY_SENSE_PIN1, INPUT);
  pinMode(BATTERY_SENSE_PIN2, INPUT);

  Serial.print(BATTERY_MAXVOLT1); Serial.print("\t");
  Serial.print(BATTERY_MAXVOLT2); Serial.println("");

  battery_publisher_timer = millis() + BATTERY_PUBLISHER_DELAY;
  battery_sense_timer = millis() + BATTERY_SENSE_DELAY;
}



void loop() {
  
  // do internal update at faster rate
  if(millis() >= battery_sense_timer) {
    battery_sense_timer = millis() + BATTERY_SENSE_DELAY;
    // do battery1
    batteryReadValue1 = analogRead(BATTERY_SENSE_PIN1);
    battery_volt1 = (battery_volt1 * BATTERY_OLD_RATE) + ((BATTERY_MAXVOLT1 * (batteryReadValue1 * BATTERY_MAXVOLTADC / BATTERY_MAXREAD1) / BATTERY_MAXVOLTADC) * BATTERY_NEW_RATE);
    // do battery2
    batteryReadValue2 = analogRead(BATTERY_SENSE_PIN2);
    battery_volt2 = (battery_volt2 * BATTERY_OLD_RATE) + ((BATTERY_MAXVOLT2 * (batteryReadValue2 * BATTERY_MAXVOLTADC / BATTERY_MAXREAD2) / BATTERY_MAXVOLTADC) * BATTERY_NEW_RATE);
  }
  
  // do ros topic publish at slower rate
  if(millis() >= battery_publisher_timer) {
    battery_publisher_timer = millis() + BATTERY_PUBLISHER_DELAY;
    // do battery1 ros topic publish
    //battery_msg1.data = battery_volt1;
    //battery_rospub1.publish(&battery_msg1);
    // do battery1 ros topic publish
    //battery_msg2.data = battery_volt2;
    //battery_rospub2.publish(&battery_msg2);
    Serial.print(battery_volt1); Serial.print("\t");
    Serial.print(battery_volt2); Serial.println("");
  }

}
