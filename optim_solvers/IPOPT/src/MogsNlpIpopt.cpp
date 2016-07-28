#include <MogsNlpIpopt.hpp>
using namespace Ipopt;

MogsNlpIpopt::MogsNlpIpopt ()
{
}

MogsNlpIpopt::~MogsNlpIpopt ()
{
}

void MogsNlpIpopt::set_robot_url(const std::vector<mogs_string> & in)
{
    robots_url_ = in;
    robot.SetRobotFile(robots_url_[0]);


}



