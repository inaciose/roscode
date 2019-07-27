#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 int_us_msg;
ros::Publisher us_data("distance", &int_us_msg);

//Ultrasonic sensor pins definition
const int echo = 25, Trig = 24;
long duration, cm;

void setup() {
  // marking us pins to be read and write
  pinMode(Trig, OUTPUT); 
  pinMode(echo, INPUT); 
  // ros
  nh.initNode();
  nh.advertise(us_data); 
}

void loop() {
  Update_Ultra_Sonic();
  nh.spinOnce();
  delay(500);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update ultrasonic sensors through serial port

void Update_Ultra_Sonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  duration = pulseIn(echo, HIGH);
  cm = microsecondsToCentimeters(duration);
  int_us_msg.data = cm;
  us_data.publish(&int_us_msg);  
}

//Conversion of microsecond to centimeter
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
