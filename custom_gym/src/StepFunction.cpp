#include "ros/ros.h"
#include "custom_gym/StepFunction.h"
#include <iostream>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
bool stepper(custom_gym::StepFunction::Request  &req, custom_gym::StepFunction::Response &res) {
// Create a transport node.
    ignition::transport::Node node;
// Prepare the input parameters.
    ignition::msgs::WorldControl ignreq;
    ignreq.set_multi_step(1);
    ignreq.set_pause(true);

    ignition::msgs::Boolean ignrep;
    bool result;
    unsigned int timeout = 1000;
// Request the "/echo" service.
    for (int i = 0; i < req.steps; i++) {
        bool executed = node.Request("/world/diff_drive/control", ignreq, timeout, ignrep, result);
    }
    res.success = result;
    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "step_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("stepper", stepper);
    ros::spin();

    return 0;
}