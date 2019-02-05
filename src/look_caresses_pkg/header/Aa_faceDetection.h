#ifndef AA_FACEDETECTION_H
#define AA_FACEDETECTION_H

#include <ros/ros.h>
#include "opencv_apps/FaceArrayStamped.h"

class Aa_faceDetection {
private:
  bool detectedLeft;
  int detectedFaceConsec;
  ros::Publisher pubVelocity;
  ros::Subscriber subImageDetectRight;
  ros::Subscriber subImageDetectLeft;
  ros::NodeHandle nh;

  void subImgDetectCallbackRight(const opencv_apps::FaceArrayStamped &msgFacesRight);
  void subImgDetectCallbackLeft(const opencv_apps::FaceArrayStamped &msgFacesLeft);
  void subTopics();
  void unsubTopics();

public:
  Aa_faceDetection (int argc, char **argv);
  int main();
};

#endif // AA_FACEDETECTION_H
