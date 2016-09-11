#include <MogsNlpIpopt.hpp>
using namespace Ipopt;

MogsNlpIpopt::MogsNlpIpopt ()
{
}

MogsNlpIpopt::~MogsNlpIpopt ()
{
}

void MogsNlpIpopt::set_robots(const std::vector<MogsRobotProperties*> & in)
{
    robots_ = in;
}



