#ifndef CloseToMiddleFAD_1_4Critere_HPP_
#define CloseToMiddleFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include <fadiff.h>
#include "AbstractFAD_1_4Critere.hpp"
#include "CloseToMiddleCriteria.hpp"
#include "MogsNlpIpopt.hpp"

class CloseToMiddleFAD_1_4Critere: public AbstractFAD_1_4Critere, CloseToMiddleCriteria
{
 public:
	CloseToMiddleFAD_1_4Critere (QDomElement critere,
                           MogsKinematics<Number> *kin);

    ~CloseToMiddleFAD_1_4Critere ();

    Number compute( const Number *x , MogsKinematics<Number> * kin, bool* compute_kin)
    {
        return CloseToMiddleCriteria::compute<Number>(x,kin, compute_kin);
    }

    F<Number>  compute( const F<Number>  *x , MogsKinematics<F<Number> > * kin, bool* compute_kin)
    {
        return CloseToMiddleCriteria::compute<F<Number> >(x,kin, compute_kin);
    }


};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
