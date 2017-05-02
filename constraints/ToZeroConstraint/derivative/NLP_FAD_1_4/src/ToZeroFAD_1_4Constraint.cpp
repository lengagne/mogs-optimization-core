#include <ToZeroFAD_1_4Constraint.hpp>

ToZeroFAD_1_4Constraint::ToZeroFAD_1_4Constraint (QDomElement constraint,
                           MogsKinematics<double> *kin):ToZeroConstraint(constraint,kin)
{

}

ToZeroFAD_1_4Constraint::~ToZeroFAD_1_4Constraint ()
{

}

extern "C" ToZeroFAD_1_4Constraint* create(QDomElement constraint, MogsKinematics<double> *kin)
{
    return new TozeroFAD_1_4Constraint(constraint, kin);
}

extern "C" void destroy(ToZeroFAD_1_4Constraint* p)
{
    delete p;
}
