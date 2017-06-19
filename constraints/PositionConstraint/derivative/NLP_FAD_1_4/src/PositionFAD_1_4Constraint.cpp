#include <PositionFAD_1_4Constraint.hpp>

PositionFAD_1_4Constraint::PositionFAD_1_4Constraint (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns):PositionConstraint(constraint,dyns)
{

}

PositionFAD_1_4Constraint::~PositionFAD_1_4Constraint ()
{

}

extern "C" PositionFAD_1_4Constraint* create(QDomElement constraint, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new PositionFAD_1_4Constraint(constraint, dyns);
}

extern "C" void destroy(PositionFAD_1_4Constraint* p)
{
    delete p;
}
