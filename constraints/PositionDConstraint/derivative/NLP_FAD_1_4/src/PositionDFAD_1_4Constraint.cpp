#include <PositionDFAD_1_4Constraint.hpp>

PositionDFAD_1_4Constraint::PositionDFAD_1_4Constraint (QDomElement constraint,
                           std::vector<MogsDynamics<double> *>& dyns):PositionDConstraint(constraint,dyns)
{

}

PositionDFAD_1_4Constraint::~PositionDFAD_1_4Constraint ()
{

}

extern "C" PositionDFAD_1_4Constraint* create(QDomElement constraint, std::vector<MogsDynamics<double> *>& dyns)
{
    return new PositionDFAD_1_4Constraint(constraint, dyns);
}

extern "C" void destroy(PositionDFAD_1_4Constraint* p)
{
    delete p;
}
