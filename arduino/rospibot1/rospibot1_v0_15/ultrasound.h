
// ultrasound config
/*
#define US_PIN_TRIGGER1 24
#define US_PIN_ECHO1 25
#define US_MAX_DISTANCE1 170
// tmp vars for reading function
long ultrasound_duration;
int ultrasound_distance;
// ultrasounds publisher timer
#define ULTRASOUND_PUBLISHER_DELAY 30
unsigned long ultrasound_publisher_timer;
*/

// BOF: SONAR
//#include "Kalman.h"
#include <NewPing.h>
#define SONAR_SCALE         1            // 1 for cm / 10 for mm 
#define SONAR_NUM           1             // Number of sensors.
#define SONAR_ZERO_MIN      200           // number of consecutive zero readings to use zero
#define SONAR_MAX_DISTANCE  200           // Maximum distance (in cm) to ping.
#define SONAR_PING_INTERVAL 33            // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SONAR_MAX_ANOMALY   33 
unsigned long sonarPingTimer[SONAR_NUM];  // Holds the times when the next ping should happen for each sensor.
unsigned int sonarRead[SONAR_NUM];        // Where the raw ping distances are stored.
unsigned int sonarKalman[SONAR_NUM];      // Where the kalman processed ping distances are stored.
unsigned int sonarValue[SONAR_NUM];       // Where computed current ping distances are stored.
unsigned int sonarZeroCounter[SONAR_NUM]; // Where the consecutive zero ping distances count are stored.
byte sonarAnomalyCounter[SONAR_NUM]; 
uint8_t sonarIndex = 0;                   // Holds the current sonar sensor number

// Sensor object array
NewPing sonar[SONAR_NUM] = {
  // Each sensor's trigger pin, echo pin, and max distance to ping. 
  //NewPing(4, 4, SONAR_MAX_DISTANCE),
  //NewPing(7, 7, SONAR_MAX_DISTANCE), 
  //NewPing(8, 8, SONAR_MAX_DISTANCE), 
  NewPing(24, 25, SONAR_MAX_DISTANCE)
};
Kalman sonarKalman0(0.05,0.05,1023,100);
//Kalman sonarKalman1(0.05,0.05,1023,100);
//Kalman sonarKalman2(0.05,0.05,1023,100);
//Kalman sonarKalman3(0.05,0.05,1023,100);
// EOF: SONAR

// ultrasounds publisher timer
#define ULTRASOUND_PUBLISHER_DELAY 30
unsigned long ultrasound_publisher_timer;

ros::Time sonarValueTs[SONAR_NUM];
uint8_t sonarIndexLast = 0;               // Holds the last available reading sonar number
boolean sonarReadAvailable = false;
