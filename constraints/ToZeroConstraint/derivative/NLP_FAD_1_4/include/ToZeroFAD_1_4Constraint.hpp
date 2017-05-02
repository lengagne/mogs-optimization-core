#ifndef ToZeroFAD_1_4Constraint_HPP_
#define ToZeroFAD_1_4Constraint_HPP_

#include <NLP_FAD_1_4.hpp>
#include "AbstractFAD_1_4Constraint.hpp"
#include "ToZeroConstraint.hpp"

class ToZeroFAD_1_4Constraint: public AbstractFAD_1_4Constraint, ToZeroConstraint
{
 public:
	ToZeroFAD_1_4Constraint (QDomElement constraint,
                           MogsKinematics<Number> *kin);

    ~ToZeroFAD_1_4Constraint();

    Number compute( const Number *x , MogsKinematics<Number> * kin, bool* compute_kin)
    {
        return ToZeroConstraint::compute<Number>(x,kin, compute_kin);
    }

    F<Number>  compute( const F<Number>  *x , MogsKinematics<F<Number> > * kin, bool* compute_kin)
    {
        return ToZeroConstraint::compute<F<Number> >(x,kin, compute_kin);
    }
};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
