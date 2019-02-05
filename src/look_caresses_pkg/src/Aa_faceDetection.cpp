#include <cstdlib>
#include <iostream>
#include "look_caresses_pkg/platform_control.h"
#include "../header/Aa_faceDetection.h"

Aa_faceDetection::Aa_faceDetection(int argc, char **argv){
  detectedLeft = false;
  detectedFaceConsec = 0;
  ros::init(argc, argv, "A_faceDetection");
  pubVelocity = nh.advertise<look_caresses_pkg::platform_control>("/miro/rob01/platform/control",100);

}

void Aa_faceDetection::subImgDetectCallbackRight(const opencv_apps::FaceArrayStamped &msgFacesRight){

  look_caresses_pkg::platform_control msgVel;

  if (msgFacesRight.faces.empty()) { //no detection with right camera
    if(detectedLeft) {
      //turn slowly clockwise to catch face also with right camera
      msgVel.body_vel.angular.z = 0.1;
      detectedFaceConsec = 0; //reset counter time for detect face
    }
    else {
      //turn a little faster clockwise because neither left camera has detected face
      msgVel.body_vel.angular.z = 0.2;
      detectedFaceConsec = 0;
    }
  }
  else {
    if(detectedLeft) {
      //face detected with both cameras, keep position
      msgVel.body_vel.angular.z = 0;
      detectedFaceConsec++; // to check the time the face is in range
    }
    else {
      //turn slowly counterclockwise to catch face also with left camera
      msgVel.body_vel.angular.z = -0.1;
      detectedFaceConsec = 0;
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
    subTopics();

    ros::Rate loop_rate(10); //10 hz
    while (detectedFaceConsec <= 30){ //detect face and keep it in focus for 30*10hz = 3 seconds
      ros::spinOnce();
      loop_rate.sleep();
    }

    unsubTopics();

    return 0;
}

void Aa_faceDetection::subTopics(){

  subImageDetectRight = nh.subscribe("/right/face_detection/faces", 1000, &Aa_faceDetection::subImgDetectCallbackRight, this);
  subImageDetectLeft = nh.subscribe("/left/face_detection/faces", 1000, &Aa_faceDetection::subImgDetectCallbackLeft, this);

}

void Aa_faceDetection::unsubTopics(){
  subImageDetectRight.shutdown();
  subImageDetectLeft.shutdown();
}

