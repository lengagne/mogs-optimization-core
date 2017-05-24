#ifndef KinematicContactFAD_1_4Constraint_HPP_
#define KinematicContactFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "KinematicContactConstraint.hpp"

class KinematicContactFAD_1_4Constraint: public AbstractFAD_1_4Constraint, KinematicContactConstraint
{
 public:
	KinematicContactFAD_1_4Constraint (QDomElement constraint,
                                std::vector<MogsOptimDynamics<double> *>& dyns);

    ~KinematicContactFAD_1_4Constraint();

    void compute(Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        KinematicContactConstraint::compute<Number>(g, dyns);
    }

    void compute( F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        KinematicContactConstraint::compute<F<Number> >(g, dyns);
    }

    void compute( Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        KinematicContactConstraint::compute<Dependency>(g, dyns);
    }

};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
