// transforms at current time publishers
geometry_msgs::TransformStamped t;
geometry_msgs::TransformStamped us_t1;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";

// ultrasound
char us0_frame[] = "/ultrasound0";
char us1_frame[] = "/ultrasound1";
char us2_frame[] = "/ultrasound2";
char us3_frame[] = "/ultrasound3";
char us4_frame[] = "/ultrasound4";
char us5_frame[] = "/ultrasound5";
char us6_frame[] = "/ultrasound6";
