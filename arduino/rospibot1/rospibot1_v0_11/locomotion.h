
// motor control config
#define EN_L 6
#define IN1_L 51
#define IN2_L 50
 
#define EN_R 7
#define IN1_R 52
#define IN2_R 53

// wheel encoders config
// Left encoder
#define Left_Encoder_PinA 2
#define Left_Encoder_PinB 22
volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;
// Right Encoder
#define Right_Encoder_PinA 3
#define Right_Encoder_PinB 23
volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;

// wheel encoders publisher timer
#define ENCODERS_PUBLISHER_DELAY 50
unsigned long encoder_publisher_timer;

// wheel speed
//double w_r=0, w_l=0;
 
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0325, wheel_sep = 0.20;
double speed_ang=0, speed_lin=0;

byte leftMotorPwmOut = 0;
byte rightMotorPwmOut = 0;

// motion inputs
volatile signed long encoderLeftPulses = 0;         // pulses since last PID computation
volatile signed long encoderLeftTotalPulses = 0;    // total pulses since motion start (use: calculate error left-right)
//signed long encoderLeftPulsesLast = 0;              // last total pulses (used to calculate encoderLeftPulses)
volatile signed long encoderRightPulses = 0;        // pulses since last PID computation
volatile signed long encoderRightTotalPulses = 0;   // total pulses since motion start
//signed long encoderRightPulsesLast = 0;             // last total pulses (used to calculate encoderRightPulses)
