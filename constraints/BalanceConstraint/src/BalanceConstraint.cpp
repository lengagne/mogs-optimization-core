#include <BalanceConstraint.hpp>

BalanceConstraint::BalanceConstraint (  QDomElement ele,
                                       std::vector<MogsOptimDynamics<double> *>& dyns)
{
    qDebug()<<"Constructor of BalanceConstraint";
    m=0;
    for (int i=0;i<dyns.size();i++)
        if( dyns[i]->model->is_robot_floating_base())
            m+=6;
    for (int i=0;i<m;i++)
    {
        upper_.push_back(0);
        lower_.push_back(0);
    }
}

BalanceConstraint::~BalanceConstraint ()
{
}
