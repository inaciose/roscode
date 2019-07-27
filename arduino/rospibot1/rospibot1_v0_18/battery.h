// battery sensor config
#define BATTERY_MAXVOLTADC 5.0
#define BATTERY_OLD_RATE 0.8
#define BATTERY_NEW_RATE 0.2
// bat1
#define BATTERY_SENSE_PIN1 A1
#define BATTERY_MAXREAD1 1024.0
#define BATTERY_MAXVOLT1 14.3 //(12.6V have a voltage divisor)
int batteryReadValue1 = 0;
float battery_volt1 = BATTERY_MAXVOLT1 / 2.0;
// bat2
#define BATTERY_SENSE_PIN2 A0
#define BATTERY_MAXREAD2 1024.0
#define BATTERY_MAXVOLT2 4.9 //(4.1V less than 5V)
float battery_volt2 = BATTERY_MAXVOLT2 / 2.0;
int batteryReadValue2 = 0;

// battery sensors publisher timer
#define BATTERY_PUBLISHER_DELAY 5000
#define BATTERY_SENSE_DELAY 500
unsigned long battery_publisher_timer;
unsigned long battery_sense_timer;
