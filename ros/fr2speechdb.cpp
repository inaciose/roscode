//
// subscribe to opencv_apps face_recognition output topic
// and send the person label to google text to speech for
// speech sound retrieval. that is later played using soundplay
// the vocalization is better than the festival method but
// need internet access
//

#include <ros/ros.h>
#include <opencv_apps/FaceArrayStamped.h>
#include "std_msgs/String.h"
#include <string>

#define LAST_FACE_ARRAY_TIME 2

//using namespace std;

double lastFaceArrayTimer;

// declare publisher globaly to be used in callback
ros::Publisher textPub;

void faceCallback(opencv_apps::FaceArrayStamped msg) {

    int nfaces = msg.faces.size();
    if(nfaces < 1) return;
    
    ROS_INFO("faces: %d", nfaces); // debug

    if(ros::Time::now().toSec() < lastFaceArrayTimer) return;
    unsigned char charstr[256];
    std::string saystring;

    for(int f = 0; f < nfaces; f++) {
        std::string sample = msg.faces[f].label;
        std::copy( sample.begin(), sample.end(), charstr );
        charstr[sample.length()] = 0;
        saystring.append("olá ");
        saystring.append(sample);
        ROS_INFO("face: %s (%f)", charstr, msg.faces[f].confidence); // debug
        std_msgs::String textStr;
        textStr.data = saystring;
        textPub.publish(textStr);
    }

    lastFaceArrayTimer = ros::Time::now().toSec() + LAST_FACE_ARRAY_TIME;
}

int main( int argc, char* argv[] ) {
  ros::init( argc, argv, "readface" );

  ros::NodeHandle n;
  ros::NodeHandle pn( "~" );

  // setup ros subscribers
  ros::Subscriber left_pwm_sub = n.subscribe("/face_recognition/output", 100, faceCallback);

  // setup ros publishers
  textPub = n.advertise<std_msgs::String>("/speech", 5);

  std_msgs::String textStr;
  textStr.data = "olá eu sou o m1p1 mais conhecido como o espantado!";
  textPub.publish(textStr);

  lastFaceArrayTimer = ros::Time::now().toSec() + LAST_FACE_ARRAY_TIME;

  while( ros::ok() ) {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
