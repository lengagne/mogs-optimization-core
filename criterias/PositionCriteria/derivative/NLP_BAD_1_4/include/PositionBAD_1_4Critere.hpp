#ifndef PositionBAD_1_4Critere_HPP_
#define PositionBAD_1_4Critere_HPP_

#include <NLP_BAD_1_4.hpp>
#include "PositionCriteria.hpp"

class PositionBAD_1_4Critere: public AbstractBAD_1_4Critere, PositionCriteria
{
 public:
	PositionBAD_1_4Critere (QDomElement critere,
                           MogsKinematics<Number> *kin);

    ~PositionBAD_1_4Critere ();

    Number compute( const Number *x , MogsKinematics<Number> * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<Number>(x,kin, compute_kin);
    }

    B<Number>  compute( const B<Number>  *x , MogsKinematics<B<Number> > * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<B<Number> >(x,kin, compute_kin);
    }


};
#endif // PositionBAD_1_4Critere_HPP_INCLUDED
