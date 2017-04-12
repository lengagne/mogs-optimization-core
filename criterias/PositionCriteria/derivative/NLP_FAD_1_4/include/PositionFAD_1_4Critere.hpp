#ifndef PositionFAD_1_4Critere_HPP_
#define PositionFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "AbstractFAD_1_4Critere.hpp"
#include "PositionCriteria.hpp"

class PositionFAD_1_4Critere: public AbstractFAD_1_4Critere, PositionCriteria
{
 public:
	PositionFAD_1_4Critere (QDomElement critere,
                           MogsKinematics<Number> *kin);

    ~PositionFAD_1_4Critere ();

    Number compute( const Number *x , MogsKinematics<Number> * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<Number>(x,kin, compute_kin);
    }

    F<Number>  compute( const F<Number>  *x , MogsKinematics<F<Number> > * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<F<Number> >(x,kin, compute_kin);
    }


};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
