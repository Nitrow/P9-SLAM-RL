#include "ros/ros.h"
#include <iostream>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

int main(int argc, char **argv)
{
    ignition::msgs::Entity robot;

    robot.set_id(40);

    ignition::transport::Node node;

    ignition::msgs::Boolean ignrep;
    bool result;
    unsigned int timeout = 1000;

    bool executed = node.Request("/world/diff_drive/remove", robot, timeout, ignrep, result);
    
    return 0;
}
