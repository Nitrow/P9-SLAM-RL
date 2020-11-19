#include <boost/python.hpp>
#define BOOST_PYTHON_STATIC_LIB
#include <ignition/msgs.hh>
#include <ignition/transport.hh>


bool step(const int &steps)
{
    ignition::transport::Node node;
    ignition::msgs::Boolean response_msg;
    ignition::msgs::WorldControl request_msg;
    request_msg.set_pause(true);
    request_msg.set_multi_step(steps);


    bool result;
    bool executed = node.Request("/world/diff_drive/control", request_msg, 1000, response_msg, result);
    return true;
}

BOOST_PYTHON_MODULE(ign_py)
{
    using namespace boost::python;
    def("step", step);
}

