#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
//#include "std_msgs/Int16.h"
#include <sensor_msgs/Range.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

// VL53L0X address id
#define I2C_ADDR_DEV 0x29

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

    // Setup high speed measurement
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
    ROS_INFO("Read status / distance: %i / %i", data->RangeStatus, data->RangeMilliMeter);

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
    //std_msgs::Int16 msg;
    sensor_msgs::Range msg;

    ros::init(argc, argv, "sensor1_range_node");
    ros::NodeHandle nh;
    //ros::Publisher range_pub = nh.advertise<std_msgs::Int16>("laser_data", 10);
    ros::Publisher range_pub = nh.advertise<sensor_msgs::Range>("laser_data", 10);

    ros::Rate loop_rate(40);

    // ros sensor_msgs Range setup
    char device_frame[] = "laser_frame";
    msg.radiation_type = sensor_msgs::Range::INFRARED;
    msg.header.frame_id = device_frame;
    msg.field_of_view = 0.20;  // fake?
    msg.min_range = 0.04;
    msg.max_range = 2.0;

    ROS_INFO("VL53L0X sensor1_node start");

    setup_vl53l0x(&vl53l0x_device);

    while (ros::ok()){
        // read VL53L0X
        device_status = read_vl53l0x_sensor(&vl53l0x_device, &device_read);

	// device is ok?
	if(device_status == VL53L0X_ERROR_NONE) {
	    // publish only quality readings
	    // if(!device_read.RangeStatus) {
	        //msg.data = device_read.RangeMilliMeter;
		msg.range = device_read.RangeMilliMeter / 1000.0;
		msg.header.stamp = ros::Time::now();
	        range_pub.publish(msg);
	    //}
	} else {
            ROS_INFO("VL53L0X ERROR");
            print_pal_error(device_status);
	}
        ros::spinOnce();
	loop_rate.sleep();
    }
 
    VL53L0X_i2c_close();
    printf("VL53L0X sensor1_node stop\n");
    return (0);
}
