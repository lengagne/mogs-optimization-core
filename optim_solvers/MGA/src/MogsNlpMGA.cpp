#include <MogsNlpMGA.hpp>

MogsNlpMGA::MogsNlpMGA ()
{
    std::cout<<"Construction of MogsNlpMGA"<<std::endl;
}

MogsNlpMGA::~MogsNlpMGA ()
{
}
////
//void MogsNlpMGA::set_robot_url(const std::vector<mogs_string> & in)
//{
//    robots_url_ = in;
//    robot.SetRobotFile(robots_url_[0]);
//
//
//}


void MogsNlpMGA::get_problem_info(unsigned int & nb_variables,
                                  unsigned int & nb_objectives,
                                  unsigned int & nb_constraints,
                                  std::vector<double>& seuils,
                                  std::vector<double>& min_var,
                                  std::vector<double>& max_var)
{

}

void MogsNlpMGA::evaluate(  std::vector<optim_infos> &infos)
{

}

void MogsNlpMGA::finalize_solution( optim_infos &info)
{

}




