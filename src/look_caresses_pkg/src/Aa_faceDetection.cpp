#include <cstdlib>
#include <iostream>
#include "look_caresses_pkg/platform_control.h"
#include "../header/Aa_faceDetection.h"

Aa_faceDetection::Aa_faceDetection(int argc, char **argv){
  detectedLeft = false;
  ros::init(argc, argv, "A_faceDetection");
  ros::NodeHandle nh;
  pubVelocity = nh.advertise<look_caresses_pkg::platform_control>("/miro/rob01/platform/control",100);
  subImageDetectRight = nh.subscribe("/right/face_detection/faces", 1000, &Aa_faceDetection::subImgDetectCallbackRight, this);
  subImageDetectLeft = nh.subscribe("/left/face_detection/faces", 1000, &Aa_faceDetection::subImgDetectCallbackLeft, this);

}

void Aa_faceDetection::subImgDetectCallbackRight(const opencv_apps::FaceArrayStamped &msgFacesRight){

  look_caresses_pkg::platform_control msgVel;

  if (msgFacesRight.faces.empty()) {
    if(detectedLeft) {
      msgVel.body_vel.angular.z = 0.1;
    }
    else {
      msgVel.body_vel.angular.z = 0.2;
    }
  }
  else {
    if(detectedLeft) {
      msgVel.body_vel.angular.z = 0;
    }
    else {
      msgVel.body_vel.angular.z = -0.1;
    }
  }

  pubVelocity.publish(msgVel);
}

void Aa_faceDetection::subImgDetectCallbackLeft(const opencv_apps::FaceArrayStamped &msgFacesLeft){

  if (msgFacesLeft.faces.empty()) {
    detectedLeft = false;
  }
  else {
    detectedLeft = true;
  }
}

int Aa_faceDetection::main()
{
    ros::spin();

    return 0;
}
