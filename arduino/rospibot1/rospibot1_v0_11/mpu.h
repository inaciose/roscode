// mpu sensor config
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data 

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);
#define OUTPUT_READABLE_QUATERNION
#define MPU_INT_PIN 11
#define MPU_PUBLISHER_DELAY 20
unsigned long mpu_publisher_timer;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
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
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true;}
