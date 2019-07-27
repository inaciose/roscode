
// motor control config
#define EN_L 6
#define IN1_L 51
#define IN2_L 50
 
#define EN_R 7
#define IN1_R 52
#define IN2_R 53

// Encoder LEFT pins definition
#define ENCODER_LEFT_FUNCTION encoderLeftCounter
#define ENCODER_LEFT_PINB 2 //B pin -> the interrupt pin 2/3
#define ENCODER_LEFT_PINA 22 //A pin -> the digital pin 9/10
#define ENCODER_LEFT_SIGNAL CHANGE
byte encoderLeftPinLast; // control
volatile long encoderLeftPulses; // the number of pulses
volatile long encoderLeftPulsesSpeedPID; // the number of pulses for PID
volatile long encoderLeftPulsesSteeringPID; // the number of pulses for PID, must be reset at same time
boolean encoderLeftDirection; // the rotation direction 

// Encoder RIGHT pins definition
#define ENCODER_RIGHT_FUNCTION encoderRightCounter
#define ENCODER_RIGHT_PINB 3 //B pin -> the interrupt pin 2/3
#define ENCODER_RIGHT_PINA 23 //A pin -> the digital pin 9/10
#define ENCODER_RIGHT_SIGNAL CHANGE
byte encoderRightPinLast; // control
volatile long encoderRightPulses; // the number of pulses
volatile long encoderRightPulsesSpeedPID; // the number of pulses PID
volatile long encoderRightPulsesSteeringPID; // the number of pulses for PID, must be reset at same time
boolean encoderRightDirection; // the rotation direction

// wheel encoders publisher timer
#define ENCODERS_PUBLISHER_DELAY 50
unsigned long encoder_publisher_timer;
 
// kinematics config
// wheel_rad is the wheel radius 
// wheel_sep is
// units are m, m/s, radian/s
#define ENCODER_PULSES 1920.0
double wheel_rad = 0.034, wheel_sep = 0.200;
double speed_ang=0, speed_lin=0;
// stored for fast calculation
double wheel_per = 2 * PI * wheel_rad;

byte leftMotorPwmOut = 0;
byte rightMotorPwmOut = 0;
