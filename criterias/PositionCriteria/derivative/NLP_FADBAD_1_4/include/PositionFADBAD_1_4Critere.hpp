#ifndef PositionFADBAD_1_4Critere_HPP_
#define PositionFADBAD_1_4Critere_HPP_

#include <NLP_FADBAD_1_4.hpp>
#include <fadiff.h>
#include "AbstractFADBAD_1_4Critere.hpp"
#include "PositionCriteria.hpp"
#include "MogsNlpIpopt.hpp"

class PositionFADBAD_1_4Critere: public AbstractFADBAD_1_4Critere, PositionCriteria
{
 public:
	PositionFADBAD_1_4Critere (QDomElement critere,
                           MogsKinematics<Number> *kin);

    ~PositionFADBAD_1_4Critere ();

    Number compute( const Number *x , MogsKinematics<Number> * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<Number>(x,kin, compute_kin);
    }

    F<Number>  compute( const F<Number>  *x , MogsKinematics<F<Number> > * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<F<Number> >(x,kin, compute_kin);
    }


};
#endif // PositionFADBAD_1_4Critere_HPP_INCLUDED
