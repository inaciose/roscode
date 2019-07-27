// ultrasound publishers
sensor_msgs::Range usrange_msg0;
ros::Publisher usrange_rospub0( "/ultrasound0", & usrange_msg0);

sensor_msgs::Range usrange_msg1;
ros::Publisher usrange_rospub1( "/ultrasound1", & usrange_msg1);

sensor_msgs::Range usrange_msg2;
ros::Publisher usrange_rospub2( "/ultrasound2", & usrange_msg2);

sensor_msgs::Range usrange_msg3;
ros::Publisher usrange_rospub3( "/ultrasound3", & usrange_msg3);

sensor_msgs::Range usrange_msg4;
ros::Publisher usrange_rospub4( "/ultrasound4", & usrange_msg4);

sensor_msgs::Range usrange_msg5;
ros::Publisher usrange_rospub5( "/ultrasound5", & usrange_msg5);

sensor_msgs::Range usrange_msg6;
ros::Publisher usrange_rospub6( "/ultrasound6", & usrange_msg6);

// wheel encoders publishers
std_msgs::Int32 encoder_msg1;
ros::Publisher encoder_rospub1("encl", &encoder_msg1);

std_msgs::Int32 encoder_msg2;
ros::Publisher encoder_rospub2("encr", &encoder_msg2);

// imu publishers
#ifdef USE_IMU
  std_msgs::Float32 imu_msg1;
  ros::Publisher imu_rospub1("imu1", &imu_msg1);
  
  std_msgs::Float32 imu_msg2;
  ros::Publisher imu_rospub2("imu2", &imu_msg2);
  
  std_msgs::Float32 imu_msg3;
  ros::Publisher imu_rospub3("imu3", &imu_msg3);
  
  #ifdef OUTPUT_READABLE_QUATERNION
    std_msgs::Float32 imu_msg4;
    ros::Publisher imu_rospub4("imu4", &imu_msg4);
  #endif
#endif

// battery sensor publishers
std_msgs::Float32 battery_msg1;
ros::Publisher battery_rospub1("battery1", &battery_msg1);

std_msgs::Float32 battery_msg2;
ros::Publisher battery_rospub2("battery2", &battery_msg2);
