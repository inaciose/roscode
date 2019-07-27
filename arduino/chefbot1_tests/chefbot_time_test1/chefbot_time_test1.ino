#include <ros.h>
#include <std_msgs/Int32.h>
#include <limits.h>

ros::NodeHandle nh;

std_msgs::Int32 int_time_msg1;
ros::Publisher time_data1("time1", &int_time_msg1);

std_msgs::Int32 int_time_msg2;
ros::Publisher time_data2("time2", &int_time_msg2);

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables

unsigned long LastUpdateMicrosecs = 0;    
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;


void setup() {
  nh.initNode();
  nh.advertise(time_data1);
  nh.advertise(time_data2);
}

void loop() {
  Update_Time();
  nh.spinOnce();
  delay(1000);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update time function
void Update_Time() {
        
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0) {
    MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;
  }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  //Serial.print("t");
  //Serial.print("\t");
  //Serial.print(LastUpdateMicrosecs);
  //Serial.print("\t");
  //Serial.print(SecondsSinceLastUpdate);
  //Serial.print("\n");

  int_time_msg1.data = LastUpdateMicrosecs;
  time_data1.publish(&int_time_msg1); 

  int_time_msg2.data = SecondsSinceLastUpdate;
  time_data2.publish(&int_time_msg2); 
}
