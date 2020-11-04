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
    bool rs_result;
    unsigned int timeout = 1000;
    
    // Deleting of model
    bool ex = node.Request("/world/diff_drive/remove", robot_remove, timeout, ignrep, result);
    
    // A little time-delay before the next request. 
    usleep(100000); // Time in MICROseconds, 100000 = 0.1 seconds 
    
    //Spawing of model 
    bool executed = node.Request("/world/diff_drive/create", robot, timeout, ignrep, rs_result);
    
    res.deletion_success = result;
    res.respawn_success = rs_result; 
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
