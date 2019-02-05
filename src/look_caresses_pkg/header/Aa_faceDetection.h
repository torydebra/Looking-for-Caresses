#ifndef AA_FACEDETECTION_H
#define AA_FACEDETECTION_H

#include <ros/ros.h>
#include "opencv_apps/FaceArrayStamped.h"
#include "look_caresses_pkg/platform_control.h"

class Aa_faceDetection {
private:
  bool detectedLeft;
  int detectedFaceConsec;
  ros::Publisher pubPlat;
  ros::Subscriber subImageDetectRight;
  ros::Subscriber subImageDetectLeft;
  ros::NodeHandle nh;

  void subImgDetectCallbackRight(const opencv_apps::FaceArrayStamped &msgFacesRight);
  void subImgDetectCallbackLeft(const opencv_apps::FaceArrayStamped &msgFacesLeft);
  void subTopics();
  void unsubTopics();

public:
  Aa_faceDetection (ros::NodeHandle nh, ros::Publisher pubPlat);
  int main();
};

#endif // AA_FACEDETECTION_H
