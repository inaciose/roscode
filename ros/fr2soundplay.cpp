#include <ros/ros.h>
#include <opencv_apps/FaceArrayStamped.h>
#include <sound_play/sound_play.h>
#include <string>

#define LAST_FACE_ARRAY_TIME 2

using namespace std;

double lastFaceArrayTimer;

void faceCallback(opencv_apps::FaceArrayStamped msg) {

    int nfaces = msg.faces.size();
    if(nfaces < 1) return;
    
    ROS_INFO("faces: %d", nfaces); // debug

    if(ros::Time::now().toSec() < lastFaceArrayTimer) return;
    sound_play::SoundClient sc;
    unsigned char charstr[256];
    std::string saystring;

    for(int f = 0; f < nfaces; f++) {
        std::string sample = msg.faces[f].label;
        std::copy( sample.begin(), sample.end(), charstr );
        charstr[sample.length()] = 0;
        saystring.append("Ola ");
        saystring.append(sample);
        ROS_INFO("face: %s (%f)", charstr, msg.faces[f].confidence); // debug
        sc.say(saystring, "voice_kal_diphone");
    }

    lastFaceArrayTimer = ros::Time::now().toSec() + LAST_FACE_ARRAY_TIME;
}

int main( int argc, char* argv[] ) {
  ros::init( argc, argv, "readface" );

  ros::NodeHandle n;
  ros::NodeHandle pn( "~" );

  // setup ros subscribers
  ros::Subscriber left_pwm_sub = n.subscribe("/face_recognition/output", 100, faceCallback);

  sound_play::SoundClient sc;
  ros::Duration(1.0).sleep(); // sleep for a second
  sc.say("Olá eu sou o m1p1 mais conhecido como o espantado!");

  lastFaceArrayTimer = ros::Time::now().toSec() + LAST_FACE_ARRAY_TIME;

  while( ros::ok() ) {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
