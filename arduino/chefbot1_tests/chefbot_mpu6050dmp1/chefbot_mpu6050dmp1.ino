//Make a pin into an interrupt
#include <PinChangeInterrupt.h>

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data 

#include <ArduinoHardware.h>
#include <ros.h>

#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 int_imu_msg1;
ros::Publisher imu_data1("imu1", &int_imu_msg1);

std_msgs::Int16 int_imu_msg2;
ros::Publisher imu_data2("imu2", &int_imu_msg2);

std_msgs::Int16 int_imu_msg3;
ros::Publisher imu_data3("imu3", &int_imu_msg3);

std_msgs::Int16 int_imu_msg4;
ros::Publisher imu_data4("imu4", &int_imu_msg4);

///////////////////////////////////////////////////////////////////////////////////////
//Creating MPU6050 Object
MPU6050 accelgyro(0x68);
#define OUTPUT_READABLE_QUATERNION
#define PUSH2 11
//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];

//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];

// ISR
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  Setup_MPU6050();
  // ros
  nh.initNode();
  nh.advertise(imu_data1);
  nh.advertise(imu_data2);   
  nh.advertise(imu_data3);
  nh.advertise(imu_data4); 
}

void loop() {
  Update_MPU6050();
  nh.spinOnce();
  delay(10);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU6050 function

void Setup_MPU6050() {


  Wire.begin();
  // initialize device
  //Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  //Initialize DMP in MPU 6050
  Setup_MPU6050_DMP();
 
}

//Setup MPU 6050 DMP
void Setup_MPU6050_DMP() {
  
   //DMP Initialization
  
   devStatus = accelgyro.dmpInitialize();
   
   accelgyro.setXGyroOffset(220);
   accelgyro.setXGyroOffset(76);
   accelgyro.setXGyroOffset(-85); 
   accelgyro.setXGyroOffset(1788);  
  
  
   if(devStatus == 0){
    
    accelgyro.setDMPEnabled(true);
    
    pinMode(PUSH2,INPUT_PULLUP);    
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PUSH2), dmpDataReady, RISING);
    
    mpuIntStatus = accelgyro.getIntStatus();
    
    dmpReady = true;
    
    packetSize = accelgyro.dmpGetFIFOPacketSize();
     
   } else {
     ;
   }
  
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050 functions
void Update_MPU6050() {
  
  //int16_t ax, ay, az;
  //int16_t gx, gy, gz;

  ///Update values from DMP for getting rotation vector
  Update_MPU6050_DMP();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050 DMP functions

void Update_MPU6050_DMP() {
  
 //DMP Processing

  if (!dmpReady) return;
    
  while (!mpuInterrupt && fifoCount < packetSize) {
      mpuInterrupt = true;
//        ;    

  }

  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();
  
  //get current FIFO count
  fifoCount = accelgyro.getFIFOCount();
  
  
  if ((mpuIntStatus & 0x10) || fifoCount > 512) {
      // reset so we can continue cleanly
      accelgyro.resetFIFO();
  }


  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        
         
        //Serial.print("i");Serial.print("\t");
        //Serial.print(q.x); Serial.print("\t");
        //Serial.print(q.y); Serial.print("\t");
        //Serial.print(q.z); Serial.print("\t");
        //Serial.print(q.w);
        //Serial.print("\n");

        int_imu_msg1.data = q.x * 100;
        imu_data1.publish(&int_imu_msg1); 

        int_imu_msg2.data = q.y * 100;
        imu_data2.publish(&int_imu_msg2); 
        
        int_imu_msg3.data = q.z * 100;
        imu_data3.publish(&int_imu_msg3); 

        int_imu_msg4.data = q.w * 100;
        imu_data4.publish(&int_imu_msg4);
                
    #endif

    #ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetEuler(euler, &q);
        //Serial.print("euler\t");
        //Serial.print(euler[0] * 180/M_PI);
        //Serial.print("\t");
        //Serial.print(euler[1] * 180/M_PI);
        //Serial.print("\t");
        //Serial.println(euler[2] * 180/M_PI);
        
        int_imu_msg1.data = euler[0] * 180/M_PI;
        imu_data1.publish(&int_imu_msg1); 
        
        int_imu_msg2.data = euler[1] * 180/M_PI;
        imu_data2.publish(&int_imu_msg2); 

        int_imu_msg3.data = qeuler[2] * 180/M_PI;
        imu_data3.publish(&int_imu_msg3); 

    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //Serial.print("ypr\t");
        //Serial.print(ypr[0] * 180/M_PI);
        //Serial.print("\t");
        //Serial.print(ypr[1] * 180/M_PI);
        //Serial.print("\t");
        //Serial.println(ypr[2] * 180/M_PI);

        int_imu_msg1.data = ypr[0] * 180/M_PI;
        imu_data1.publish(&int_imu_msg1); 
        
        int_imu_msg2.data = ypr[1] * 180/M_PI;
        imu_data2.publish(&int_imu_msg2); 

        int_imu_msg3.data = ypr[2] * 180/M_PI;
        imu_data3.publish(&int_imu_msg3); 
        
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetAccel(&aa, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        //Serial.print("areal\t");
        //Serial.print(aaReal.x);
        //Serial.print("\t");
        //Serial.print(aaReal.y);
        //Serial.print("\t");
        //Serial.println(aaReal.z);

        int_imu_msg1.data = aaReal.x;
        imu_data1.publish(&int_imu_msg1); 
        
        int_imu_msg2.data = aaReal.y;
        imu_data2.publish(&int_imu_msg2); 

        int_imu_msg3.data = aaReal.z;
        imu_data3.publish(&int_imu_msg3); 
        
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetAccel(&aa, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        //Serial.print("aworld\t");
        //Serial.print(aaWorld.x);
        //Serial.print("\t");
        //Serial.print(aaWorld.y);
        //Serial.print("\t");
        //Serial.println(aaWorld.z);

        int_imu_msg1.data = aaWorld.x;
        imu_data1.publish(&int_imu_msg1); 
        
        int_imu_msg2.data = aaWorld.y;
        imu_data2.publish(&int_imu_msg2); 

        int_imu_msg3.data = aaWorld.z;
        imu_data3.publish(&int_imu_msg3); 
        
    #endif

  }
     
}
