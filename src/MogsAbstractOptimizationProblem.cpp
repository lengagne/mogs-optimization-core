#include "MogsAbstractOptimizationProblem.h"

void MogsAbstractOptimizationProblem::solve()
{
	std::cout<<"MogsAbstractOptimizationProblem::solve()"<<std::endl;

    std::cout<<"init_problem"<<std::endl;
    solver_->set_root(root_);
    solver_->set_robots(robots_);
    solver_->init_problem(&pb_);
    std::cout<<"init_problem done"<<std::endl;

    #ifdef MogsVisu_FOUND
    if (solver_->need_visu( ))
    {
        visu_optim_ = new VisuHolder("MogsOptimization");
        visu_optim_->init_from_problem(this);
        solver_->set_visu(visu_optim_,true);
    }
    #endif // MogsVisu_FOUND

    std::cout<<"MogsAbstractOptimizationProblem::prepare"<<std::endl;
    solver_->prepare(pb_);
    std::cout<<"MogsAbstractOptimizationProblem::prepare done "<<std::endl;
    solver_->solve(pb_);
}
