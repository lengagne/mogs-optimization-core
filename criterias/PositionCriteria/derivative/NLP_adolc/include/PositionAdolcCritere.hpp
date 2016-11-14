#ifndef PositionAdolcCritere_HPP_
#define PositionAdolcCritere_HPP_

#include <adolc.h>
#include "AbstractAdolcCritere.hpp"
#include "PositionCriteria.hpp"

class PositionAdolcCritere: public AbstractAdolcCritere, PositionCriteria
{
 public:
	PositionAdolcCritere (QDomElement critere,
                           MogsKinematics<double> *kin);

    ~PositionAdolcCritere ();

    double compute( const double *x , MogsKinematics<double> * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<double>(x,kin, compute_kin);
    }

    adouble compute( const adouble *x , MogsKinematics<adouble> * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<adouble>(x,kin, compute_kin);
    }


};
#endif // PositionAdolcCritere_HPP_INCLUDED
