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

    void compute( const Number *x , Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns, bool* compute_kin)
    {
        ToZeroConstraint::compute<Number>(x,g, dyns, compute_kin);
    }

    void compute( const F<Number>  *x , F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns, bool* compute_kin)
    {
        ToZeroConstraint::compute<F<Number> >(x,g, dyns, compute_kin);
    }

    void compute( const Dependency  *x , Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns, bool* compute_kin)
    {
        ToZeroConstraint::compute<Dependency>(x,g, dyns, compute_kin);
    }

};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
