#include <KinematicContactFAD_1_4Constraint.hpp>

KinematicContactFAD_1_4Constraint::KinematicContactFAD_1_4Constraint (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns):KinematicContactConstraint(constraint,dyns)
{

}

KinematicContactFAD_1_4Constraint::~KinematicContactFAD_1_4Constraint ()
{

}

extern "C" KinematicContactFAD_1_4Constraint* create(QDomElement constraint, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new KinematicContactFAD_1_4Constraint(constraint, dyns);
}

extern "C" void destroy(KinematicContactFAD_1_4Constraint* p)
{
    delete p;
}
