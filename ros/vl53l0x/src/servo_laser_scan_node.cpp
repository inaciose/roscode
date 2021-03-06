#include <wiringPi.h>
#include <softServo.h>
#include <ros/ros.h>

#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

// servo config
#define SERVO_PIN 1
#define SERVO_DEGRES 100.0
#define SERVO_PULSE_MIN -250
#define SERVO_PULSE_MAX 1250
#define DEG_TO_RAD 0.0174532925

// VL53L0X address id
#define I2C_ADDR_DEV 0x29
#define VL53L0X_NUM_READINGS 100
#define VL53L0X_READINGS_FREQUENCY 0.25

// VL53L0X required firmware version
#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 1

void print_pal_error(VL53L0X_Error device_status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(device_status, buf);
    ROS_INFO("VL53L0X_API status: %i : %s", device_status, buf);
}

VL53L0X_Error init_vl53l0x(VL53L0X_Dev_t *pvl53l0x_device) {
    VL53L0X_Error device_status = VL53L0X_ERROR_NONE;
    uint8_t isApertureSpads, VhvSettings, PhaseCal;
    uint32_t refSpadCount;

    if(device_status == VL53L0X_ERROR_NONE) {
        ROS_INFO("VL53L0X_StaticInit");
        device_status = VL53L0X_StaticInit(pvl53l0x_device);
        print_pal_error(device_status);
    }

    if(device_status == VL53L0X_ERROR_NONE) {
        ROS_INFO("VL53L0X_PerformRefCalibration");
        device_status = VL53L0X_PerformRefCalibration(pvl53l0x_device, &VhvSettings, &PhaseCal);
        print_pal_error(device_status);
    }

    if(device_status == VL53L0X_ERROR_NONE) {
        ROS_INFO("VL53L0X_PerformRefSpadManagement");
        device_status = VL53L0X_PerformRefSpadManagement(pvl53l0x_device, &refSpadCount, &isApertureSpads);
        ROS_INFO("refSpadCount = %d, isApertureSpads = %d", refSpadCount, isApertureSpads);
        print_pal_error(device_status);
    }

    if(device_status == VL53L0X_ERROR_NONE) {
        // setup continuous ranging mode (not required for VL53L0X_PerformSingleRangingMeasurement)
        ROS_INFO("VL53L0X_SetDeviceMode");
        device_status = VL53L0X_SetDeviceMode(pvl53l0x_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        print_pal_error(device_status);
    }

    // Setup high speed measuremen
    if (device_status == VL53L0X_ERROR_NONE) {
        device_status = VL53L0X_SetLimitCheckValue(pvl53l0x_device,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.25*65536));
	}

    if (device_status == VL53L0X_ERROR_NONE) {
        device_status = VL53L0X_SetLimitCheckValue(pvl53l0x_device,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(32*65536));
    }

    if (device_status == VL53L0X_ERROR_NONE) {
        device_status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pvl53l0x_device, 20000);
    }

    return device_status;
}

void setup_vl53l0x(VL53L0X_Dev_t *pvl53l0x_device) {
    VL53L0X_Error device_status = VL53L0X_ERROR_NONE;
    VL53L0X_Version_t device_version;
    VL53L0X_DeviceInfo_t device_info;
    int32_t ctl_status;

    // set device address
    pvl53l0x_device->I2cDevAddr = I2C_ADDR_DEV;

    // select i2c-0 or i2c-1
    pvl53l0x_device->fd = VL53L0X_i2c_init("/dev/i2c-1", pvl53l0x_device->I2cDevAddr);

    if (pvl53l0x_device->fd < 0) {
        device_status = VL53L0X_ERROR_CONTROL_INTERFACE;
        ROS_INFO("Failed to init\n");
    }

    // get the VL53L0X API version running in the firmware
    if(device_status == VL53L0X_ERROR_NONE) {
        ctl_status = VL53L0X_GetVersion(&device_version);
        if (ctl_status != 0) device_status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    // verify the version of the VL53L0X API running in the firmware
    if(device_status == VL53L0X_ERROR_NONE) {
        if( device_version.major != VERSION_REQUIRED_MAJOR ||
            device_version.minor != VERSION_REQUIRED_MINOR ||
            device_version.build != VERSION_REQUIRED_BUILD ) {
            ROS_INFO("VL53L0X API Version Error: firmware has %d.%d.%d (revision %d). Software requires %d.%d.%d.",
                device_version.major, device_version.minor, device_version.build, device_version.revision,
                VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
        }
    }

    // VL53L0X data initialization
    if(device_status == VL53L0X_ERROR_NONE) {
        ROS_INFO("VL53L0X_DataInit");
        device_status = VL53L0X_DataInit(pvl53l0x_device);
        print_pal_error(device_status);
    }

    // VL53L0X device information
    if(device_status == VL53L0X_ERROR_NONE) {
        device_status = VL53L0X_GetDeviceInfo(pvl53l0x_device, &device_info);
        if(device_status == VL53L0X_ERROR_NONE) {
            ROS_INFO("Device Name : %s", device_info.Name);
            ROS_INFO("Device Type : %s", device_info.Type);
            ROS_INFO("Device ID : %s", device_info.ProductId);
            ROS_INFO("ProductRevisionMajor : %d", device_info.ProductRevisionMajor);
            ROS_INFO("ProductRevisionMinor : %d", device_info.ProductRevisionMinor);

        if ((device_info.ProductRevisionMinor != 1) && (device_info.ProductRevisionMinor != 1)) {
        	ROS_INFO("Error expected cut 1.1 but found cut %d.%d",
                       device_info.ProductRevisionMajor, device_info.ProductRevisionMinor);
                device_status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
        print_pal_error(device_status);
    }

    // VL53L0X Initialize
    if(device_status == VL53L0X_ERROR_NONE) {
        device_status = init_vl53l0x(pvl53l0x_device);
    }
}

VL53L0X_Error read_vl53l0x_sensor(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *data) {

    VL53L0X_Error device_status = VL53L0X_PerformSingleRangingMeasurement(dev, data);
    //ROS_INFO("Read status / distance: %i / %i", data->RangeStatus, data->RangeMilliMeter);

    // get and display status string message
    //char buf[VL53L0X_MAX_STRING_LENGTH];
    //VL53L0X_GetRangeStatusString(data->RangeStatus, buf);
    //ROS_INFO("Status msg: %s", buf);

    return device_status;
}

int main(int argc, char **argv) {
    VL53L0X_Dev_t vl53l0x_device;
    VL53L0X_Error device_status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t device_read;
    sensor_msgs::LaserScan msg1;
    sensor_msgs::LaserScan msg2;
    float servo_pulse;

    ros::init(argc, argv, "servo_laser_scan_node");
    ros::NodeHandle nh;

    ros::Publisher range_pub1 = nh.advertise<sensor_msgs::LaserScan>("laser_scan1", 50);
    ros::Publisher range_pub2 = nh.advertise<sensor_msgs::LaserScan>("laser_scan2", 50);

    //ros::Rate loop_rate(40);

    ROS_INFO("VL53L0X sensor1_node start");

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

    // prepare ros sensor_msgs LaserRange 
    char device_frame1[] = "laser_frame1";
    char device_frame2[] = "laser_frame2";
    uint16_t num_readings = VL53L0X_NUM_READINGS;
    double laser_frequency = VL53L0X_READINGS_FREQUENCY;

    // set constant LaserRange fields
    msg1.header.frame_id = device_frame1;
/*
    msg.angle_min = -1.221730475;
    msg.angle_max = 1.221730475;
    msg.angle_increment = 2.44346095 / num_readings;
*/
    msg1.angle_min = -(SERVO_DEGRES * DEG_TO_RAD / 2);
    msg1.angle_max = SERVO_DEGRES * DEG_TO_RAD / 2;
    msg1.angle_increment = (SERVO_DEGRES * DEG_TO_RAD) / num_readings;

    msg1.time_increment = (float)(1.0 / laser_frequency) / (float)num_readings;
    msg1.scan_time = 4; //(millis()-scan_start)/1000.0;
    msg1.range_min = 0.04;
    msg1.range_max = 2.0;

    ROS_INFO("sensor_msgs LaserRange 1: %f, %f, %f, %f", msg1.angle_min, msg1.angle_max, msg1.angle_increment, msg1.time_increment);

    // set constant LaserRange fields
    msg2.header.frame_id = device_frame2;

    msg2.angle_min = SERVO_DEGRES * DEG_TO_RAD / 2;
    msg2.angle_max = -(SERVO_DEGRES * DEG_TO_RAD / 2);
    msg2.angle_increment = -(SERVO_DEGRES * DEG_TO_RAD) / num_readings;

    msg2.time_increment = (float)(1.0 / laser_frequency) / (float)num_readings;
    msg2.scan_time = 4; //(millis()-scan_start)/1000.0;
    msg2.range_min = 0.04;
    msg2.range_max = 2.0;

    ROS_INFO("sensor_msgs LaserRange 2: %f, %f, %f, %f", msg1.angle_min, msg1.angle_max, msg1.angle_increment, msg1.time_increment);

    double ranges[num_readings];
    double intensities[num_readings];

    // setup laser sensor
    setup_vl53l0x(&vl53l0x_device);

    // start servo
    float pulse = SERVO_PULSE_MIN;
    softServoWrite(SERVO_PIN, pulse);
    delay(250);

    int16_t position = 0;
    int16_t direction = 1;
    uint16_t distance = 0;
    //uint16_t f;

    while (ros::ok()){

	// move servo
	softServoWrite(SERVO_PIN, SERVO_PULSE_MIN + position * servo_pulse);
        delay(25);

	//if(direction > 0) {
            // read VL53L0X
            device_status = read_vl53l0x_sensor(&vl53l0x_device, &device_read);
            ROS_INFO("angle / pulse / status / distance: %i / %f / %i / %i", position, SERVO_PULSE_MIN + position * servo_pulse, device_read.RangeStatus, device_read.RangeMilliMeter);

            // device is ok?
            if(device_status == VL53L0X_ERROR_NONE) {
                // publish only quality readings
                //if(!device_read.RangeStatus != 2) {
                    distance = device_read.RangeMilliMeter;
                    ranges[position] = (float) distance / 1000.0;
                    intensities[position] = 0;
                //}
            } else {
                ROS_INFO("VL53L0X ERROR");
                print_pal_error(device_status);
            }
        //}

	// next position & publish
	if(direction > 0) {
            position++;
	    if(position > SERVO_DEGRES){
		// publish and reverse
                ROS_INFO("PUB 1");
                //msg.scan_time = 0.5; //(millis()-scan_start)/1000.0;
                msg1.ranges.resize(num_readings);
                msg1.intensities.resize(num_readings);
                for(unsigned int i = 0; i < num_readings; ++i){
                    msg1.ranges[i] = ranges[i];
                    msg1.intensities[i] = intensities[i];
                }
                msg1.header.stamp = ros::Time::now();

		// publish 
                range_pub1.publish(msg1);

		direction = -1;
		position = SERVO_DEGRES;
	    }
	} else {
	    position--;
	    if(position < 0) {
                // publish and reverse
                ROS_INFO("PUB 2");
                //msg.scan_time = 0.5; //(millis()-scan_start)/1000.0;
                msg2.ranges.resize(num_readings);
                msg2.intensities.resize(num_readings);
                for(unsigned int i = 0; i < num_readings; ++i){
                    msg2.ranges[i] = ranges[i];
                    msg2.intensities[i] = intensities[i];
                }
                msg2.header.stamp = ros::Time::now();

                // publish
                range_pub2.publish(msg2);

		direction = 1;
		position = 0;
	    }
	}

        ros::spinOnce();


/*
	for(f = 0; f < SERVO_DEGRES + 1; f += 1) {
	    if(f == SERVO_DEGRES + 1) {

 		ROS_INFO("PUB & recicle\n");
                //msg.scan_time = 0.5; //(millis()-scan_start)/1000.0;
                msg.ranges.resize(num_readings);
                msg.intensities.resize(num_readings);
                for(unsigned int i = 0; i < num_readings; ++i){
                    msg.ranges[i] = ranges[i];
                    msg.intensities[i] = intensities[i];
                }
                msg.header.stamp = ros::Time::now();

                range_pub.publish(msg);

		f = 0;
		position = 0;

		pulse = SERVO_PULSE_MIN;
		softServoWrite(SERVO_PIN, pulse);
		delay(250);

	    }

	//while(position < num_readings) {
	    softServoWrite(SERVO_PIN, SERVO_PULSE_MIN + f * servo_pulse);
	    delay(25);

        // read VL53L0X
        device_status = read_vl53l0x_sensor(&vl53l0x_device, &device_read);

	ROS_INFO("angle / pulse / status / distance: %i / %f / %i / %i", f, SERVO_PULSE_MIN + f * servo_pulse, device_read.RangeStatus, device_read.RangeMilliMeter);

	// device is ok?
	if(device_status == VL53L0X_ERROR_NONE) {
	    // publish only quality readings
	    //if(!device_read.RangeStatus) {
	        distance = device_read.RangeMilliMeter;
		ranges[position] = (float) distance / 1000.0;
		intensities[position] = 0;

	    //}
	} else {
            ROS_INFO("VL53L0X ERROR");
            print_pal_error(device_status);
	}

	position++;
        ros::spinOnce();

	}
*/
    }

    VL53L0X_i2c_close();
    printf("VL53L0X sensor1_node stop\n");
    return (0);
}
