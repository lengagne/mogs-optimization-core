#ifndef  POSITIONADOLCCRITERE_HPP_
#define  POSITIONADOLCCRITERE_HPP_


#include <NLP_adolc.hpp>
#include <adolc.h>
#include "AbstractAdolcCritere.hpp"
#include "MogsNlpIpopt.hpp"
#include "MogsKinematics.h"
#include "PositionCriteria.hpp"

class PositionAdolcCritere: public AbstractAdolcCritere, PositionCriteria
{   public:
	
	PositionAdolcCritere (QDomElement critere,
                          MogsKinematics<double>* kin);

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

#endif // POSITIONADOLCCRITERE_HPP_
