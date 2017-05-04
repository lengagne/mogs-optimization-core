#ifndef ToZeroFAD_1_4Constraint_HPP_
#define ToZeroFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "ToZeroConstraint.hpp"

class ToZeroFAD_1_4Constraint: public AbstractFAD_1_4Constraint, ToZeroConstraint
{
 public:
	ToZeroFAD_1_4Constraint (QDomElement constraint,
                           MogsKinematics<Number> *kin);

    ~ToZeroFAD_1_4Constraint();

    void compute( const Number *x , Number* g, MogsKinematics<Number> * kin, bool* compute_kin)
    {
        ToZeroConstraint::compute<Number>(x,g, kin, compute_kin);
    }

    void compute( const F<Number>  *x , F<Number>* g, MogsKinematics<F<Number> > * kin, bool* compute_kin)
    {
        ToZeroConstraint::compute<F<Number> >(x,g, kin, compute_kin);
    }

};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
