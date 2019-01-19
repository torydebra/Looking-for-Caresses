#ifndef A_AWAKENING_H
#define A_AWAKENING_H

#include "look_caresses_pkg/platform_sensors.h"
#include "std_msgs/Int32.h"
#include <ros/ros.h>

class A_awakening {
private:
  bool notRead;
  bool touched;
  int loneliness; //from 0 to 100
  ros::Subscriber subLoneliness;
  ros::Subscriber subTouched;
  ros::Publisher pubPlat;
  ros::Publisher pubLoneliness;

  void subTouchCallback(const look_caresses_pkg::platform_sensors &msg);
  void subLonelinessCallback(const std_msgs::Int32& msg);

public:
  A_awakening (int argc, char **argv);
  int main();
};

#endif // A_AWAKENING_H
