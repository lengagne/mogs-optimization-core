#include <PositionDFAD_1_4Constraint.hpp>

PositionDFAD_1_4Constraint::PositionDFAD_1_4Constraint (QDomElement constraint,
                           MogsKinematics<double> *kin):PositionDConstraint(constraint,kin)
{

}

PositionDFAD_1_4Constraint::~PositionDFAD_1_4Constraint ()
{

}

extern "C" PositionDFAD_1_4Constraint* create(QDomElement constraint, MogsKinematics<double> *kin)
{
    return new PositionDFAD_1_4Constraint(constraint, kin);
}

extern "C" void destroy(PositionDFAD_1_4Constraint* p)
{
    delete p;
}
