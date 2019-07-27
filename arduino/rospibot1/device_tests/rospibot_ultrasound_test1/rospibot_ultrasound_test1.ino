#define trigPin 24
#define echoPin 25

// 12 and 13 refers to the pin numbers on Arduino
// You can choose to use any pins on the Arduino for digital 
// signal read and write (which sensor uses) 

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 int_msg;
ros::Publisher arduino("distance", &int_msg);

// Standard ROS node intialization

void setup() { // Setup code runs only once
  //Serial.begin (9600);  
  // You can choose to see output on a serial monitor
  // on the computer 
  // But from what I found, Arduino can't seem to maintain
  // both serial communication to ROS and serial monitor
  pinMode(trigPin, OUTPUT); // marking pins to be read and write 
  pinMode(echoPin, INPUT); 
  nh.initNode();
  nh.advertise(arduino); 
}

void loop() { // This part of the code runs repeatedly
  long duration;
  int distance;

  digitalWrite(trigPin, LOW); // This is just setup code 
  delayMicroseconds(2);       // So that ultrasound sensor would
  digitalWrite(trigPin, HIGH);// start reading
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); 

  duration = pulseIn(echoPin, HIGH); 
  // The data coming in from ultrasound sensor is just 
  // digital data (0, 1), but it signifies distance data 
  // (0 ~ 200) by sending "pulses" of certain length 

  distance = (duration/2) / 29.1;
  int_msg.data = distance;

  if (distance >= 2000 ) {
    //Serial.println("Out of range");
  }
  else {
    //Serial.print(distance);
    //Serial.println(" cm");
    arduino.publish(&int_msg);
    nh.spinOnce();
    // Standard publish routine for ROS
  }
  delay(30); // Wait 30 ms before running again
}
