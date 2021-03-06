// battery sensor config
#define BATTERY_SENSE_PIN1 A5
#define BATTERY_MAXREAD1 1023.0
#define BATTERY_MAXVOLT1 12.0
float battery_volt1 = BATTERY_MAXVOLT1 / 2.0;
#define BATTERY_SENSE_PIN2 A6
#define BATTERY_MAXREAD2 1023.0
#define BATTERY_MAXVOLT2 5.0
float battery_volt2 = BATTERY_MAXVOLT2 / 2.0;
#define BATTERY_OLD_RATE 0.8
#define BATTERY_NEW_RATE 0.2
int batteryReadValue1 = 0;
int batteryReadValue2 = 0;

// battery sensors publisher timer
#define BATTERY_PUBLISHER_DELAY 5000
#define BATTERY_SENSE_DELAY 500
unsigned long battery_publisher_timer;
unsigned long battery_sense_timer;
