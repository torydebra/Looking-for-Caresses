#include <ros/ros.h>
#include "../header/A_awakening.h"
#include "../header/Aa_faceDetection.h"
#include "../header/B_approach.h"
#include "../header/C_interaction.h"


int main(int argc, char **argv)
{
  ROS_INFO("[COORD] Started");
  ros::init(argc, argv, "Coordinator");
  ros::NodeHandle nh;
  ros::Publisher pubLoneliness = nh.advertise<std_msgs::Int32>("miro/look4caresses/loneliness", 1000);
  std_msgs::Int32 msg;
  msg.data = 50;
  pubLoneliness.publish(msg); // initialize loneliness to 50
  ROS_INFO("[COORD] Published initial value of loneliness = 50");
  A_awakening a_awakening(argc, argv);
  Aa_faceDetection aa_faceDetection(argc, argv);
  B_approach b_approach(argc, argv);
  C_interaction c_interaction(argc, argv);

  while (ros::ok()){
    A_awakening a_awakening(argc, argv);
    ROS_INFO("[COORD] node A started");
    a_awakening.main();
    ROS_INFO("[COORD] node A finished, Starting node Aa");
    aa_faceDetection.main();
    ROS_INFO("[COORD] node Aa finished, Starting node B");
    b_approach.main();
    ROS_INFO("[COORD] node B finished, Starting node C");
    c_interaction.main();
    ROS_INFO("[COORD] node C finished, Starting node A");
  }

  ROS_INFO("[COORD] Shutting down");
  return 0;
}
