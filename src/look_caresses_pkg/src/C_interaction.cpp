#include <string>
#include "../header/C_interaction.h"

/** @brief Costructor for phase C task.

    @param nh the ros nodeHandle to subscribe
    @param pubPlat the Publisher object to publish on topic
*/
C_interaction::C_interaction(ros::NodeHandle nh, ros::Publisher pubPlat) {
  notRead = true;
  positiveApproach = 0;
  negativeApproach = 0;
  counterSound = 0;

  this->nh = nh;
  this->pubPlat = pubPlat;

}

/** @brief Callback for images from right camera

      Gesture Indexes:
      0: "CarBottomTop"
      1: "FixedBody"
      2: "PatHead"
      3: "FixedHead"
      4: "PatBody"
      5: "CarTopBottom"
      "No_Gesture"

     to approach miro, caress him on body
     to send away miro, pat on head (caresses on head would be difficult to detect)
     patBody not used due to high ratio of false positive (even when body is not touched)

    @param pattern the touch pattern arrived
*/
void C_interaction::classCallback(const std_msgs::String &pattern)
{

  if (!strcmp(pattern.data.c_str() , "CarBottomTop") || !strcmp(pattern.data.c_str() , "CarTopBottom")){
    positiveApproach++;

  } else if (!strcmp(pattern.data.c_str() , "PatHead") || !strcmp(pattern.data.c_str() , "FixedHead")){
    negativeApproach++;
  }

}


/** @brief function to send command to Miro to show happines while he is being touched.
 *
 *   Here we test some of the sound that MiRo can emit.
 *   P1 sounds are more "mammal" sounds, P2 are a sort of human pirate sound
 *
 *   P1 sound indexes:  (Nice sounds from 12 to 17)
     * 12 rrr rararar like alien
     * 13 Bad noise
     * 14 Mrrr nice
     * 15 MMM...PRRR nice
     * 16 MMMPRAO good
     * 17 mmmrrrr good
     * 18 Bad noise
     * 19 Bad noise
     * 20 Bad noise
     * 31 Bad noise

    @param loneliness the loneliness value needed to show more or less happiness
*/
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

    plat_msgs_happy.sound_index_P1 = 12 + rand()%5; // mammal alien sound
    //plat_msgs_awake.sound_index_P2 = rand()%20; //pirate sound
    counterSound = 0;
  } else {
    counterSound++;
  }

  pubPlat.publish(plat_msgs_happy);
}

/** @brief Main for Task C. The task updates the loneliness value if MiRo receive caresses,
    and make MiRo acts happily while being rubbed.

    @param loneliness the loneliness value
    @return loneliness the loneliness updated during the phase
*/
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
/**
 * @brief function to subcribe to specific topics
 */
void C_interaction::subTopics(){
  /** read loneliness value **/
  subClass = nh.subscribe("/miro/look4caresses/classifyGesture", 1000, &C_interaction::classCallback, this);

}

/**
 * @brief function to unsubcribe to topics
 */
void C_interaction::unsubTopics(){
  subClass.shutdown();
}
