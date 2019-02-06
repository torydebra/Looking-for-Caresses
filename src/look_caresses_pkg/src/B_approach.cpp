#include <iostream>
#include <string>
#include <numeric>
#include "../header/B_approach.h"

/** @brief Costructor for phase B task.

    @param nh the ros nodeHandle to subscribe
    @param pubPlat the Publisher object to publish on topic
*/
B_approach::B_approach(ros::NodeHandle nh, ros::Publisher pubPlat){

  sonarMsgs = boost::circular_buffer<float>(averageNum);
  counter = 0;
  this->nh = nh;
  this->pubPlat = pubPlat;
}

/** @brief Callback for sonar messages

    @param sensor_range the sonar range arrived
*/
void B_approach::sonarCallback(const sensor_msgs::Range &sensor_range)
{
    look_caresses_pkg::platform_control msg;
    sonarMsgs.push_front(sensor_range.range);

    if (sonarMsgs.full()){ //wait to have "averageNum" values of range

      // average between more than 1 value to avoid the errors of the sonar (sometimes returns zeros)
      float sum = std::accumulate(sonarMsgs.begin(), sonarMsgs.end(), 0.0); //0 init the sum to 0
      float average = sum/averageNum;
      ROS_INFO("[B] Sonar Range average: %f", average);

      if (average > 0.15) { // in meters
        msg.body_vel.linear.x = 30; // good choiche for the linear velocity
        msg.body_vel.angular.z = 0.05; // because miro turns unwanted
      }
      else { // miro must not move
        msg.body_vel.linear.x = 0;
        msg.body_vel.angular.z = 0;
        counter ++;
      }
    } else { // miro must not move
      msg.body_vel.linear.x = 0;
      msg.body_vel.angular.z = 0;
    }
    pubPlat.publish(msg);
}

/**
 * @brief main for approachin phase. MiRo goes straight until the sensor detect something
 * (ideally the user hand) sufficently near
 * @return 0 for correct execution
 */
int B_approach::main()
{
    counter = 0;
    sonarMsgs.erase(sonarMsgs.begin(), sonarMsgs.end());

    ROS_INFO("[B] Started");

    subTopics();

    // Publish the kinematic of the head to position the head up for detecting sonar range
    look_caresses_pkg::platform_control plat_msgs_headup;
    float body_config_headup[4] = {0.0, 0.4, 0, -0.1};
    float body_config_speed_headup[4] = {0.0, -1.0, -1.0, -1.0};
    for (int i =0; i<4; i++){
      plat_msgs_headup.body_config[i] = body_config_headup[i];
      plat_msgs_headup.body_config_speed[i] = body_config_speed_headup[i];
    }

   pubPlat.publish(plat_msgs_headup);


    ros::Rate loop_rate2(2);
    while (ros::ok() && counter<6){
      ros::spinOnce();
      loop_rate2.sleep();
    }

    //ros::spin();
    unsubTopics();
    ROS_INFO("[B] Finished");
    return 0;
}

/**
 * @brief function to subcribe to specific topics
 */
void B_approach::subTopics(){
    subRange = nh.subscribe("/miro/rob01/sensors/sonar_range", 1000, &B_approach::sonarCallback, this);
}


/**
 * @brief function to unsubcribe to topics
 */
void B_approach::unsubTopics(){
  subRange.shutdown();
}
