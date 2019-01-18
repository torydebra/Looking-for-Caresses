#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <iostream>
#include "look_caresses_pkg/platform_control.h"
#include "look_caresses_pkg/platform_sensors.h"

bool notRead = true;
bool touched = false;
int loneliness; //from 0 to 100
ros::Subscriber subLoneliness;
ros::Subscriber subTouched;


void subTouchCallback(const look_caresses_pkg::platform_sensors &msg){
  for(int i=0; i<4;i++){
    if (msg.touch_head[i] == 1 || msg.touch_body[i] == 1){
      touched = true;
      subTouched.shutdown();
      return;
    }
  }
}


void subLonelinessCallback(const std_msgs::Int32& msg)
{
    loneliness =  msg.data;
    subLoneliness.shutdown();
    notRead = false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "A_Awakening");
    ros::NodeHandle n;

    /** read loneliness value **/
    subLoneliness = n.subscribe("miro/rob01/look4caresses/loneliness", 1000, subLonelinessCallback);
    while(ros::ok() && notRead){
      ros::spinOnce();
    }
    ROS_INFO("I read Loneliness: [%d]", loneliness);

    /** SLEEPING PHASE */
    // sub to see if miro is touched while sleeping
    subTouched = n.subscribe("/miro/rob01/platform/sensors", 1000, subTouchCallback);

    // to publish kinematic and cosmetic thing to make miro look asleep
    ros::Publisher pubPlat = n.advertise<look_caresses_pkg::platform_control>
        ("/miro/rob01/platform/control", 1000);
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
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("I am awake =)");

    /** AWAKENING PHASE */

    /** update loneliness value **/
    ros::Publisher pub = n.advertise<std_msgs::Int32>("miro/rob01/look4caresses/loneliness", 1000);

    ROS_INFO("I am writing Loneliness: %d", loneliness);

    std_msgs::Int32 toSend;
    toSend.data = loneliness;
    pub.publish(toSend);

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
      ros::spinOnce();
      loop_rate2.sleep();
    }
}
