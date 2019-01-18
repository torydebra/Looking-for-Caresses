#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <string>

bool notRead = true;
int positiveApproach = 0;
int negativeApproach = 0;
int loneliness;
ros::Subscriber subLoneliness;


/*
     if index == 0:
       gesture_str = "CarBottomTop"
      elif index == 1:
        gesture_str = "FixedBody"
      elif index == 2:
        gesture_str = "PatHead"
      elif index == 3:
        gesture_str = "FixedHead"
      elif index == 4:
        gesture_str = "PatBody"
      elif index == 5:
        gesture_str = "CarTopBottom"
    else:
      gesture_str = "No_Gesture"
**/
void classCallback(const std_msgs::String &pattern)
{
  /* to approach miro, caress him on body
     to send away miro, pat on head (caresses on head would be difficult to detect)
     patBody not used due to high ratio of false positive (even when body is not touched)
  */
  if ((pattern.data == "CarBottomTop") || (pattern.data == "CarTopBottom")){
    positiveApproach++;

  } else if ((pattern.data == "PatHead") || (pattern.data == "FixedHead"))
    negativeApproach++;

}

void subLonelinessCallback(const std_msgs::Int32& msg)
{
    loneliness = msg.data;
    subLoneliness.shutdown();
    notRead = false;
}

int main(int argc, char **argv)
{
  bool miroHappy = false;

  ros::init(argc, argv, "A_Awakening");
  ros::NodeHandle nh;

  /** read loneliness value **/
  subLoneliness = nh.subscribe("miro/rob01/look4caresses/loneliness", 1000, subLonelinessCallback);
  while(ros::ok() && notRead){
    ros::spinOnce();
  }
  ROS_INFO("[C] I read Loneliness: [%d]", loneliness);

  /** INTERACTION PHASE **/
  ros::Subscriber subClass = nh.subscribe("/miro/look4caresses/classifyGesture", 1000, classCallback);
  srand(time(NULL));

  while (ros::ok()){
    /* positive approch increment at high rate: if a single caresses is given to miro,
       the callback find (7-8?) times the pattern, so we increment loneliness only when positive approach
       is sufficently high */
    if (positiveApproach > 15){
      loneliness--;
      positiveApproach = 0; //reset positive counter
    }
    if (negativeApproach > 25){ // 25 to be sure that the user DONT want to interact
      //miroHappy = false;
      break; //exit because user does not want to interact
    }
    if (loneliness < (20 - rand() % 10)) {
      //miroHappy = true;
      break; //exit because miro is satisfied
    }

    ros::spinOnce();

  }

  /** Miro back to sleep **/
  // Go back few centimeters?




  return 0;

}
