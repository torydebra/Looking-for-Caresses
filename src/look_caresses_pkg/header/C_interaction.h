#ifndef C_INTERACTION_H
#define C_INTERACTION_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

class C_interaction {
private:
  bool notRead;
  int positiveApproach;
  int negativeApproach;
  int loneliness;
  int counterSound;
  ros::Subscriber subLoneliness;
  ros::Subscriber subClass;
  ros::Publisher pubPlat;
  ros::NodeHandle nh;

  void classCallback(const std_msgs::String &pattern);
  void subLonelinessCallback(const std_msgs::Int32& msg);
  void showHappiness(ros::Publisher pubPlat);
public:
  C_interaction(int argc, char **argv);
  int main();
};

#endif // C_INTERACTION_H
