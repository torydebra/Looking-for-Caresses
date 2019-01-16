#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <iostream>
#include "look_caresses_pkg/core_control.h"
#include "look_caresses_pkg/sleep.h"

using namespace std;

int loneliness; //from 0 to 100

void subCallback(const std_msgs::Int32& msg)
{
    ROS_INFO("I am reading Loneliness: [%d]", msg.data);
    loneliness =  msg.data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "A_Awakening");
    ros::NodeHandle n;



    /** read loneliness value **/
    ros::Subscriber sub = n.subscribe("loneliness", 1000, subCallback);
    ros::spinOnce();

    /** Random Awakening **/
    ros::Rate loop_rate(1); // 1 Hz

    while(ros::ok()){

        if (loneliness > (rand() % 100)){
            break;
        }

        if (loneliness < 100){
            loneliness++;
        }
        loop_rate.sleep();
    }


//topic name	core/config
//message type	miro_msgs/core_config

    //miro_msgs::core_config

    ros::Publisher pub2 = n.advertise<look_caresses_pkg::core_control>("core/config", 1000);
    look_caresses_pkg::core_control core_msgs;
    core_msgs.sleep_drive_target.wakefulness = 1; // TODO: also put 0 se sleeping?
    pub2.publish(core_msgs);

    /** update loneliness value **/
    ros::Publisher pub = n.advertise<std_msgs::Int32>("loneliness", 1000);

    ROS_INFO("I am writing Loneliness: %d", loneliness);

    std_msgs::Int32 toSend;
    toSend.data = loneliness;
    pub.publish(toSend);

    ros::spinOnce();
    //while(1);

}
