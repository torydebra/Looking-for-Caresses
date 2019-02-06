#include <ros/ros.h>
#include "../header/A_awakening.h"
#include "../header/Aa_faceDetection.h"
#include "../header/B_approach.h"
#include "../header/C_interaction.h"
#include "look_caresses_pkg/platform_control.h"

/**
 * @brief Main ros node to perform all the task. It initialize the task and then run sequentially the specific main of
 * each task
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
  ROS_INFO("[COORD] Started");
  ros::init(argc, argv, "Coordinator");
  ros::NodeHandle nh;
  ros::Publisher pubPlat = nh.advertise<look_caresses_pkg::platform_control>
      ("/miro/rob01/platform/control", 1000);

  int loneliness = 10;   // initialize loneliness to 50

  ROS_INFO("[COORD] Published initial value of loneliness = 50");

  A_awakening a_awakening(nh, pubPlat);
  Aa_faceDetection aa_faceDetection(nh, pubPlat);
  B_approach b_approach(nh, pubPlat);
  C_interaction c_interaction(nh, pubPlat);

  while (ros::ok()){
    ROS_INFO("[COORD] Starting node A");
    loneliness = a_awakening.main(loneliness);
    ROS_INFO("[COORD] node A finished, Starting node Aa");
    aa_faceDetection.main();
    ROS_INFO("[COORD] node Aa finished, Starting node B");
    b_approach.main();
    ROS_INFO("[COORD] node B finished, Starting node C");
    loneliness = c_interaction.main(loneliness);
    ROS_INFO("[COORD] node C finished, Starting node A");
  }

  ROS_INFO("[COORD] Shutting down");
  return 0;
}
