#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "look_caresses_pkg/platform_control.h"
#include <string>

bool notRead = true;
int positiveApproach = 0;
int negativeApproach = 0;
int loneliness;
int counterSound = 0;
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

  if (!strcmp(pattern.data.c_str() , "CarBottomTop") || !strcmp(pattern.data.c_str() , "CarTopBottom")){
    positiveApproach++;

  } else if (!strcmp(pattern.data.c_str() , "PatHead") || !strcmp(pattern.data.c_str() , "FixedHead")){
    negativeApproach++;
  }

}

void subLonelinessCallback(const std_msgs::Int32& msg)
{
    loneliness = msg.data;
    subLoneliness.shutdown();
    notRead = false;
}

void showHappiness(ros::Publisher pubPlat){
  /** Kinematic and cosmetic things to show miro happy */
  look_caresses_pkg::platform_control plat_msgs_happy;

  //sligtly move ears and tails to catture user attention
  plat_msgs_happy.tail = 1 - (loneliness/100.0); //move tail faster if loneliness goes down
  //TODO continuosly moving ears
  //plat_msgs_awake.ear_rotate[0] = 0.2;
  //plat_msgs_awake.ear_rotate[1] = 0.2;


  plat_msgs_happy.eyelid_closure = 0.4;

  srand(time(NULL));

  if (counterSound == 5){ //to not send too many sound consecutevely
    /* P1 SOUND:
     * 1 Schifo
     * 2 Schifo microfono
     * 3 Schifo brop
     *
     * 11 Schifo
     * 12 rrr rararar sembra alieno carino
     * 13 Schifo
     * 14 Mrrr carino
     * 15 MMM...PRRR carino
     * 16 MMMPRAO fiero
     * 17 mmmrrrr fiero
     * 18 scorreggina schifo
     * 19 schifo
     * 20 singhiozzo schifo
     * 31 Schifo
     *
     * Nice sounds from 12 to 17
     */
    plat_msgs_happy.sound_index_P1 = 12 + rand()%5; // mammal alien sound
    //plat_msgs_awake.sound_index_P2 = rand()%20; //pirate sound
    counterSound = 0;
  } else {
    counterSound++;
  }

  pubPlat.publish(plat_msgs_happy);

}


int main(int argc, char **argv)
{
  bool miroHappy = false;

  ros::init(argc, argv, "A_Awakening");
  ros::NodeHandle nh;

  ros::Publisher pubPlat = nh.advertise<look_caresses_pkg::platform_control>
      ("/miro/rob01/platform/control", 1000);

  /** read loneliness value **/
  subLoneliness = nh.subscribe("miro/look4caresses/loneliness", 1000, subLonelinessCallback);
  while(ros::ok() && notRead){
    ros::spinOnce();
  }
  ROS_INFO("[C] I read Loneliness: [%d]", loneliness);

  /** INTERACTION PHASE **/
  ros::Subscriber subClass = nh.subscribe("/miro/look4caresses/classifyGesture", 1000, classCallback);
  srand(time(NULL));

  ros::Rate loop_rate(2); //0.5 s
  while (ros::ok()){
    /* positive approch increment at high rate: if a single caresses is given to miro,
       the callback find (7-8?) times the pattern, so we increment loneliness only when positive approach
       is sufficently high */
    if (positiveApproach > 10){
      loneliness--;
      ROS_INFO("[C] I am becoming happier :) (loneliness:%d)", loneliness);
      showHappiness(pubPlat);

      positiveApproach = 0; //reset positive counter
    }
    if (negativeApproach > 25){ // 25 to be sure that the user DONT want to interact
      //miroHappy = false;
      ROS_INFO("[C] I'm going away :'( %d", negativeApproach);
      break; //exit because user does not want to interact
    }
    if (loneliness < (20 - rand() % 10)) {
      ROS_INFO("[C] I am satisfied! (loneliness:%d)", loneliness);
      //miroHappy = true;
      break; //exit because miro is satisfied
    }

    ros::spinOnce();
    loop_rate.sleep();

  }



  /** Miro back to sleep **/
  // Go back few centimeters?





  return 0;

}

