#include "ros/ros.h"
#include <iostream>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
double x, y, z, R, P, Y;

int main(int argc, char **argv)
{
    ignition::msgs::EntityFactory robot;
    ignition::msgs::Entity robot_remove;
    
    // Setting name
    robot.set_name("Boxbot");
    
    // Setting name and type of model to remove 
    robot_remove.set_name("Boxbot");
    robot_remove.set_type(ignition::msgs::Entity_Type_MODEL);

    
    // Setting file to spawn
    robot.set_sdf_filename("/home/rasmus/Desktop/P9-SLAM-RL/src/simulations/models/cylinder.sdf");
    
    // Setting Pose
    ignition::math::Pose3d pose
	{
	    x = 0,
	    y = 0,
	    z = 2,
	    R = 0,
	    P = 0,
	    Y = 0
	};
    ignition::msgs::Set(robot.mutable_pose(), pose);


    ignition::transport::Node node;
    
    ignition::msgs::Boolean ignrep;
    bool result;
    unsigned int timeout = 1000;
    
    // Deleting of model
    bool ex = node.Request("/world/diff_drive/remove", robot_remove, timeout, ignrep, result);
    
    //Spawing of model 
    bool executed = node.Request("/world/diff_drive/create", robot, timeout, ignrep, result);
    
    return 0;
}
