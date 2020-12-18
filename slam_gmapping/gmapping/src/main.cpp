/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "slam_gmapping.h"
#include <iostream>
bool shouldReset = false;

void sysCmdCallback(const std_msgs::String& sys_cmd)
  {
    if (sys_cmd.data == "reset")
    {
      shouldReset = true;
    }
  }


void main_loop() {

  while (ros::ok()) {
    SlamGMapping gn;
    gn.startLiveSlam();

    while (!shouldReset) {
      ros::spinOnce();
      ros::Duration(0.2).sleep();
    }
    ROS_INFO("Resetting map...");
    shouldReset = false;

  }


}
int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_gmapping");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("syscommand", 10, sysCmdCallback);
  main_loop();
  ros::spin();

  return(0);
}
