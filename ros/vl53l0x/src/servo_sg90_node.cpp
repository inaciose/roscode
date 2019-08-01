#include <wiringPi.h>
#include <softServo.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>

#define SERVO_PIN 1
#define SERVO_DEGRES 140.0
#define SERVO_PULSE_MIN -250
#define SERVO_PULSE_MAX 1250

float servo_pulse;

void subCallback(const std_msgs::Int16 msg) {
	ROS_INFO("got: %i", msg.data);
	float pulse = SERVO_PULSE_MIN + (msg.data - 20) * servo_pulse; 
	softServoWrite(SERVO_PIN, pulse);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "servo_sg90_node");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("servo_angle", 10, subCallback);

	//ros::Rate loop_rate(100);

	// setup the wiringPi library
    	if (wiringPiSetup() < 0) {
        	ROS_INFO("wiringPiSetup error: exit");
        	exit(EXIT_FAILURE);
    	}

	// setup the servo funtions library
	softServoSetup (SERVO_PIN, -1, -1, -1, -1, -1, -1, -1) ;

	// set pulse per degree global variable
	servo_pulse = (float)(abs(SERVO_PULSE_MIN) + abs(SERVO_PULSE_MAX)) / SERVO_DEGRES;
	ROS_INFO("pulse %f", servo_pulse);

	ros::spin();

	//int f;
	//while ( ros::ok() ) {

/*
		// with angle convert
                softServoWrite(SERVO_PIN, -250);
                delay(2000);
                for(f = 0; f < 160; f += 1) {
                        softServoWrite(SERVO_PIN, SERVO_PULSE_MIN + f * servo_pulse);
			ROS_INFO("pulses %f", SERVO_PULSE_MIN + f * servo_pulse);
                        delay (25) ;
                }
                delay(2000);

                for(f = 160; f > 0 ; f -= 1) {
                        softServoWrite(SERVO_PIN, SERVO_PULSE_MIN + f * servo_pulse);
			ROS_INFO("pulses %f", SERVO_PULSE_MIN + f * servo_pulse);
                        delay (25) ;
                }

*
/*
                // no angle convert
                softServoWrite (SERVO_PIN, -250);
                delay(100);
                for(f = -250; f < 1250; f += 50) {
                        softServoWrite (SERVO_PIN,  f);
                        delay (25) ;
                }
                //delay(2000);

                for(f = 1250; f > -250 ; f -= 50) {
                        softServoWrite (SERVO_PIN,  f);
                        delay (25) ;
                }
*/
        	// spin & sleep
        	//ros::spinOnce();
        	//loop_rate.sleep();
	//}

	return 0;
}
