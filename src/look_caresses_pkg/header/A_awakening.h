#ifndef A_AWAKENING_H
#define A_AWAKENING_H

#include "look_caresses_pkg/platform_sensors.h"
#include "look_caresses_pkg/platform_control.h"
#include <ros/ros.h>

class A_awakening {
private:
  bool notRead;
  bool touched;
  ros::Subscriber subTouched;
  ros::NodeHandle nh;
  ros::Publisher pubPlat;

  void subTouchCallback(const look_caresses_pkg::platform_sensors &msg);
  void subTopics();
  void unsubTopics();

public:
  A_awakening (ros::NodeHandle nh, ros::Publisher pubPlat);
  int main(int loneliness);
};

#endif // A_AWAKENING_H
