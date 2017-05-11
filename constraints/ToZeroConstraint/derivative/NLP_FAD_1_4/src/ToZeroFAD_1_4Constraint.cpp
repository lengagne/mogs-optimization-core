#include <ToZeroFAD_1_4Constraint.hpp>

ToZeroFAD_1_4Constraint::ToZeroFAD_1_4Constraint (QDomElement constraint,
                           std::vector<MogsDynamics<double> *> dyns):ToZeroConstraint(constraint,dyns)
{

}

ToZeroFAD_1_4Constraint::~ToZeroFAD_1_4Constraint ()
{

}

extern "C" ToZeroFAD_1_4Constraint* create(QDomElement constraint, std::vector<MogsDynamics<double> *> dyns)
{
    return new ToZeroFAD_1_4Constraint(constraint, dyns);
}

extern "C" void destroy(ToZeroFAD_1_4Constraint* p)
{
    delete p;
}
