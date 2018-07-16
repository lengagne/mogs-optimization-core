
#include "MogsAbstractOptimization.h"

void MogsAbstractOptimization::read_problem(const mogs_string & filename)
{
    MogsAbstractProblem::read_problem(filename);
    #ifdef MogsVisu_FOUND
    visu_during_optim_ = false;
    QDomElement ElVisuDuring=root_.firstChildElement("visu_during_optim");
    if (!ElVisuDuring.isNull())
    {
        visu_during_optim_ = convert_to_bool(ElVisuDuring.text().simplified());
    }
    #endif // MogsVisu_FOUND
}


void MogsAbstractOptimization::solve()
{
    std::cout<<"MogsAbstractOptimization::solve()"<<std::endl;
#ifdef MogsVisu_FOUND
    qDebug()<<"MogsVisu found";
	visu_optim_ = new VisuHolder();
	visu_optim_->init_from_problem(this);
#else
    qDebug()<<"MogsVisu not found";
#endif
std::cout<<"MogsAbstractOptimization::solve() end"<<std::endl;
}
