#ifndef  POSITIONFADBAD_1_4CRITERE_HPP_
#define  POSITIONFADBAD_1_4CRITERE_HPP_


#include <NLP_FADBAD_1_4.hpp>
#include <fadiff.h>
#include "AbstractFADBAD_1_4Critere.hpp"
#include "MogsNlpIpopt.hpp"
#include "MogsKinematics.h"
#include "PositionCriteria.hpp"

class PositionFADBAD_1_4Critere: public AbstractFADBAD_1_4Critere, PositionCriteria
{   public:

	PositionFADBAD_1_4Critere (QDomElement critere,
                          MogsKinematics<double>* kin);

    ~PositionFADBAD_1_4Critere ();

    double compute( const double *x , MogsKinematics<double> * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<double>(x,kin, compute_kin);
    }

    F<double>  compute( const F<double>  *x , MogsKinematics<F<double> > * kin, bool* compute_kin)
    {
        return PositionCriteria::compute<F<double> >(x,kin, compute_kin);
    }
};

#endif // POSITIONFADBAD_1_4CRITERE_HPP_
