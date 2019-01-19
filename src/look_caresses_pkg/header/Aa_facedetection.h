#ifndef AA_FACEDETECTION_H
#define AA_FACEDETECTION_H

#include <ros/ros.h>
#include "opencv_apps/FaceArrayStamped.h"

class Aa_faceDetection {
private:
  bool detectedLeft;
  ros::Publisher pubVelocity;
  ros::Subscriber subImageDetectRight;
  ros::Subscriber subImageDetectLeft;

  void subImgDetectCallbackRight(const opencv_apps::FaceArrayStamped &msgFacesRight);
  void subImgDetectCallbackLeft(const opencv_apps::FaceArrayStamped &msgFacesLeft);
public:
  Aa_faceDetection (int argc, char **argv);
  int main();
};

#endif // AA_FACEDETECTION_H
