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
    
    // Setting name
    robot.set_name("Boxbot");
    
    // Setting file
    
    robot.set_sdf_filename("/home/rasmus/Desktop/P9-SLAM-RL/src/simulations/models/cylinder.sdf");
    
    // Setting Pose
    ignition::math::Pose3d pose
	{
	    x = 0,
	    y = 0,
	    z = 0,
	    R = 0,
	    P = 0,
	    Y = 0
	};
    ignition::msgs::Set(robot.mutable_pose(), pose);


    ignition::transport::Node node;

    ignition::msgs::Boolean ignrep;
    bool result;
    unsigned int timeout = 1000;

    bool executed = node.Request("/world/diff_drive/create", robot, timeout, ignrep, result);
    
    return 0;
}
