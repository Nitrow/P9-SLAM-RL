#include "ros/ros.h"
#include "custom_gym/spawner.h"
#include <iostream>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <unistd.h>
double x, y, z, R, P, Y;

bool respawn(custom_gym::spawner::Request &req, custom_gym::spawner::Response &res)
{
    ignition::msgs::EntityFactory robot;
    ignition::msgs::Entity robot_remove;
    
    ignition::msgs::Boolean response_msg;
    ignition::msgs::Pose set_pose;
    
    set_pose.set_name("vechicle_blue");
    set_pose.set_id(40);
    set_pose.mutable_position()->set_x(0.0);
    set_pose.mutable_position()->set_y(0.0);
    set_pose.mutable_position()->set_z(0.3);
    set_pose.mutable_orientation()->set_x(0.0);
    set_pose.mutable_orientation()->set_y(0.0);
    set_pose.mutable_orientation()->set_z(0.0);
    set_pose.mutable_orientation()->set_w(1.0);
    
    


    ignition::transport::Node node;
    
    ignition::msgs::Boolean ignrep;
    bool result;
   
    
    bool setting_pose = node.Request("/world/diff_drive/set_pose", set_pose, 1000, response_msg, result);
    
    
    return true;    
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_respawn");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("/respawn", respawn);
    ros::spin();

    
    return 0;
}
