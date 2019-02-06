#include <string>
#include "../header/C_interaction.h"

C_interaction::C_interaction(ros::NodeHandle nh, ros::Publisher pubPlat) {
  notRead = true;
  positiveApproach = 0;
  negativeApproach = 0;
  counterSound = 0;

  this->nh = nh;
  this->pubPlat = pubPlat;

}

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
void C_interaction::classCallback(const std_msgs::String &pattern)
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


void C_interaction::showHappiness(int loneliness){
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


int C_interaction::main(int loneliness)
{
  notRead = true;
  positiveApproach = 0;
  negativeApproach = 0;
  counterSound = 0;

  ROS_INFO("[C] Started");

  bool miroHappy = false;
  subTopics();

  ROS_INFO("[C] I have a Loneliness value of: [%d]", loneliness);

  look_caresses_pkg::platform_control plat_msgs_caresses;
  float body_config_caresses[4] = {0.0, 0.85, -0.25, -0.25}; // head down
  float body_config_speed_caresses[4] = {0.0, -1.0, -1.0, -1.0};
  for (int i =0; i<4; i++){
    plat_msgs_caresses.body_config[i] = body_config_caresses[i];
    plat_msgs_caresses.body_config_speed[i] = body_config_speed_caresses[i];
  }

  ros::Rate loop_rate3(1);
  for (int i =0; i<3; i++){
    plat_msgs_caresses.sound_index_P1 = 12 + rand()%5; // mammal alien sound
    pubPlat.publish(plat_msgs_caresses);
    loop_rate3.sleep();
  }

  /** INTERACTION PHASE **/
  srand(time(NULL));

  ros::Rate loop_rate(2); //0.5 s
  while (ros::ok()){
    /* positive approch increment at high rate: if a single caresses is given to miro,
       the callback find (7-8?) times the pattern, so we increment loneliness only when positive approach
       is sufficently high */
    if (positiveApproach > 10){
      loneliness--;
      ROS_INFO("[C] I am becoming happier :) (loneliness:%d)", loneliness);
      showHappiness(loneliness);

      positiveApproach = 0; //reset positive counter
    }
    if (negativeApproach > 25){ // 25 to be sure that the user DONT want to interact
      //miroHappy = false;
      ROS_INFO("[C] I'm going away :'( (loneliness: %d)", loneliness);
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

  unsubTopics();


  /** Miro back to sleep **/
  look_caresses_pkg::platform_control msg;
  msg.body_vel.linear.x = -30; // good choiche for the linear velocity
  msg.body_vel.angular.z = -0.05; // because miro turns unwanted
  ros::Rate loop_rate2(1);
  for (int i = 0; i<4; i++){
     pubPlat.publish(msg);
     loop_rate2.sleep();
  }

  ROS_INFO("[C] Finished");
  return loneliness;

}

void C_interaction::subTopics(){
  /** read loneliness value **/
  subClass = nh.subscribe("/miro/look4caresses/classifyGesture", 1000, &C_interaction::classCallback, this);

}

void C_interaction::unsubTopics(){
  subClass.shutdown();
}
