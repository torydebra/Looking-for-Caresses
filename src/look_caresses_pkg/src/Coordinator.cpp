#include <ros/ros.h>
#include "../header/A_awakening.h"

int main(int argc, char **argv)
{

  A_awakening a_awakening(argc, argv);
  a_awakening.main();

  ROS_INFO("Hello world!");
}
