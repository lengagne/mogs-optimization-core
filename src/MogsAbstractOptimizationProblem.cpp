#include "MogsAbstractOptimizationProblem.h"

void MogsAbstractOptimizationProblem::solve()
{
	std::cout<<"MogsAbstractOptimizationProblem::solve()"<<std::endl;

    std::cout<<"init_problem"<<std::endl;
    optim_solver_->set_root(root_);
    optim_solver_->set_robots(robots_);
    optim_solver_->init_problem(&pb_);
    std::cout<<"init_problem done"<<std::endl;

    #ifdef MogsVisu_FOUND
    if (optim_solver_->need_visu( ))
    {
        visu_optim_ = new VisuHolder("MogsOptimization");
        visu_optim_->init_from_problem(this);
        optim_solver_->set_visu(visu_optim_,true);
    }
    #endif // MogsVisu_FOUND

    std::cout<<"MogsAbstractOptimizationProblem::prepare"<<std::endl;
    optim_solver_->prepare(pb_);
    std::cout<<"MogsAbstractOptimizationProblem::prepare done "<<std::endl;
    optim_solver_->solve(pb_);
}
