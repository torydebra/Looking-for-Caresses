#ifndef C_INTERACTION_H
#define C_INTERACTION_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "look_caresses_pkg/platform_control.h"

class C_interaction {
private:
  bool notRead;
  int positiveApproach;
  int negativeApproach;
  int counterSound;
  ros::NodeHandle nh;
  ros::Subscriber subClass;
  ros::Publisher pubPlat;

  void classCallback(const std_msgs::String &pattern);
  void showHappiness(int loneliness);
  void subTopics();
  void unsubTopics();

public:
  C_interaction(ros::NodeHandle nh, ros::Publisher pubPlat);
  int main(int loneliness);
};

#endif // C_INTERACTION_H
