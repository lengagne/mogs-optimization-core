#include <BalanceFAD_1_4Constraint.hpp>

BalanceFAD_1_4Constraint::BalanceFAD_1_4Constraint (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns):BalanceConstraint(constraint,dyns)
{

}

BalanceFAD_1_4Constraint::~BalanceFAD_1_4Constraint ()
{

}

extern "C" BalanceFAD_1_4Constraint* create(QDomElement constraint, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new BalanceFAD_1_4Constraint(constraint, dyns);
}

extern "C" void destroy(BalanceFAD_1_4Constraint* p)
{
    delete p;
}
