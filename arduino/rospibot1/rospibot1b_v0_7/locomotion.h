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
//long encoderLeftPulsesLast; // the number of last pulses recorded for odometry
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
//long encoderRightPulsesLast; // the number of last pulses recorded for odometry
boolean encoderRightDirection; // the rotation direction

// encoders distance targets
enum e_encoderPulsesTargetStates {ENC_TARGET_DISABLE, ENC_TARGET_READY, ENC_TARGET_ENABLED};
byte encoderPulsesTargetState = ENC_TARGET_DISABLE; 
long encoderLeftPulsesTarget = 0;
long encoderRightPulsesTarget = 0;
boolean encoderLeftPulsesOnTarget = false;
boolean encoderRightPulsesOnTarget = false;

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
double distancePerPulse = wheel_per / ENCODER_PULSES;
double pulses_per_m = 1.0 / wheel_per * ENCODER_PULSES;

// odometry
//double bodyX = 0;
//double bodyY = 0;
//double bodyTheta = 0;

byte leftMotorPwmOut = 0;
byte rightMotorPwmOut = 0;

enum e_baseMotionStates { MOTION_IDLE, MOTION_STOP, MOTION_FORWARD, MOTION_BACKWARD, MOTION_ROTATE_LEFT, MOTION_ROTATE_RIGHT, MOTION_FORWARD_CURVE_LEFT, MOTION_FORWARD_CURVE_RIGHT, MOTION_BACKWARD_CURVE_LEFT, MOTION_BACKWARD_CURVE_RIGHT};
int baseMotionState = MOTION_IDLE;
