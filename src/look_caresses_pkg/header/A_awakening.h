#include "look_caresses_pkg/platform_control.h"
#include "look_caresses_pkg/platform_sensors.h"
#include "std_msgs/Int32.h"

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
