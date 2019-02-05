#include "std_msgs/String.h"
#include <cstdlib>
#include <iostream>
#include "../header/A_awakening.h"

A_awakening::A_awakening(ros::NodeHandle nh, ros::Publisher pubPlat){
  notRead = true;
  touched = false;
  this->nh = nh;
  this->pubPlat = pubPlat;
}


void A_awakening::subTouchCallback(const look_caresses_pkg::platform_sensors &msg){
  for(int i=0; i<4;i++){
    if (msg.touch_head[i] == 1 || msg.touch_body[i] == 1){
      touched = true;
      ROS_INFO("[A] Hey, you woke me up!");
      return;
    }
  }
}


int A_awakening::main(int loneliness)
{

    notRead = true;
    touched = false;

    ROS_INFO("[A] Started");
    subTopics();

    ROS_INFO("[A] I have a Loneliness value of: [%d]", loneliness);


    /** SLEEPING PHASE */

    // to publish kinematic and cosmetic thing to make miro look asleep
    ROS_INFO("[A] I go to sleep");
    look_caresses_pkg::platform_control plat_msgs_sleeping;
    plat_msgs_sleeping.eyelid_closure = 0.8;
    float body_config_sleeping[4] = {0.0, 1.0, 0, -0.1}; // head down
    float body_config_speed_sleeping[4] = {0.0, -1.0, -1.0, -1.0};
    for (int i =0; i<4; i++){
      plat_msgs_sleeping.body_config[i] = body_config_sleeping[i];
      plat_msgs_sleeping.body_config_speed[i] = body_config_speed_sleeping[i];
    }

    /** Random Awakening **/
    ros::Rate loop_rate(2); // 1 Hz
    srand(time(NULL));
    while(ros::ok() && !touched){

        if (loneliness > (50 + rand() % 50)){
            break;
        }

        if (loneliness < 100){
            loneliness++;
        }
        pubPlat.publish(plat_msgs_sleeping);
        ros::spinOnce(); // need to receive touchedCallback
        ROS_INFO("[A] Sleeping... loneliness = %d", loneliness);
        loop_rate.sleep();
    }

    ROS_INFO("[A] I am awake =) with Loneliness value of: %d", loneliness);

    /** AWAKENING PHASE */

    /** Kinematic and cosmetic things to show miro awake */
    look_caresses_pkg::platform_control plat_msgs_awake;
    plat_msgs_awake.eyelid_closure = 0.1;

    //put miro head s.t. camera can work properly for face detection
    float body_config_awake[4] = {0.0, 0.6, 0, -0.1};
    float body_config_speed_awake[4] = {0.0, -1.0, -1.0, -1.0};
    for (int i =0; i<4; i++){
      plat_msgs_awake.body_config[i] = body_config_awake[i];
      plat_msgs_awake.body_config_speed[i] = body_config_speed_awake[i];
    }
    //sligtly move ears and tails to catture user attention
    plat_msgs_awake.tail = -0.2;
    plat_msgs_awake.ear_rotate[0] = 0.2;
    plat_msgs_awake.ear_rotate[1] = 0.2;
    srand(time(NULL));

    //publish cosmetic thing to show awakening
    ros::Rate loop_rate2(1);
    for (int i =0; i<5; i++){
      plat_msgs_awake.blink_time = 5 + rand()%20;
      //plat_msgs_awake.sound_index_P1 = rand()%32; // mammal alien sound
      plat_msgs_awake.sound_index_P2 = rand()%20; //pirate sound
      pubPlat.publish(plat_msgs_awake);
      //ros::spinOnce();
      loop_rate2.sleep();
    }

    unsubTopics();

    ROS_INFO("[A] Finished");
    return loneliness;
}


void A_awakening::subTopics(){

  //Subscribed topics

  // sub to see if miro is touched while sleeping
  subTouched = nh.subscribe("/miro/rob01/platform/sensors", 1000, &A_awakening::subTouchCallback, this);
}


void A_awakening::unsubTopics(){
  subTouched.shutdown();
}
