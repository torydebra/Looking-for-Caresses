#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include "opencv_apps/FaceArrayStamped.h"
#include "look_caresses_pkg/platform_control.h"

ros::Publisher pubVelocity;
bool detectedLeft = false;

void subImgDetectCallbackRight(const opencv_apps::FaceArrayStamped &msgFacesRight){

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

void subImgDetectCallbackLeft(const opencv_apps::FaceArrayStamped &msgFacesLeft){

  if (msgFacesLeft.faces.empty()) {
    detectedLeft = false;
  }
  else {
    detectedLeft = true;
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "A_faceDetection");
    ros::NodeHandle n;

    pubVelocity = n.advertise<look_caresses_pkg::platform_control>("/miro/rob01/platform/control",100);
    ros::Subscriber subImageDetectRight = n.subscribe("/right/face_detection/faces", 1000, subImgDetectCallbackRight);
    ros::Subscriber subImageDetectLeft = n.subscribe("/left/face_detection/faces", 1000, subImgDetectCallbackLeft);

    ros::spin();

    return 0;
}
