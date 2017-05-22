#ifndef ToZeroFAD_1_4Constraint_HPP_
#define ToZeroFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "ToZeroConstraint.hpp"

class ToZeroFAD_1_4Constraint: public AbstractFAD_1_4Constraint, ToZeroConstraint
{
 public:
	ToZeroFAD_1_4Constraint (QDomElement constraint,
                                std::vector<MogsOptimDynamics<double> *>& dyns);

    ~ToZeroFAD_1_4Constraint();

    void compute(Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        ToZeroConstraint::compute<Number>(g, dyns);
    }

    void compute( F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        ToZeroConstraint::compute<F<Number> >(g, dyns);
    }

    void compute( Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        ToZeroConstraint::compute<Dependency>(g, dyns);
    }

};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
