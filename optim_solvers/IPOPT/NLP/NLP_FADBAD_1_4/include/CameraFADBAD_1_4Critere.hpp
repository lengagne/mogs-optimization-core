#ifndef CameraFADBAD_1_4Critere_HPP_
#define CameraFADBAD_1_4Critere_HPP_

#include <NLP_FADBAD_1_4.hpp>
#include <fadiff.h>
#include "AbstractFADBAD_1_4Critere.hpp"
#include "CameraCriteria.hpp"
#include "MogsNlpIpopt.hpp"

class CameraFADBAD_1_4Critere: public AbstractFADBAD_1_4Critere, CameraCriteria
{
 public:
	CameraFADBAD_1_4Critere (QDomElement critere,
                           MogsKinematics<double> *kin);

    ~CameraFADBAD_1_4Critere ();

    double compute( const double *x , MogsKinematics<double> * kin, bool* compute_kin)
    {
        return CameraCriteria::compute<double>(x,kin, compute_kin);
    }

    F<double>  compute( const F<double>  *x , MogsKinematics<F<double> > * kin, bool* compute_kin)
    {
        return CameraCriteria::compute<F<double> >(x,kin, compute_kin);
    }


};
#endif // CameraFADBAD_1_4Critere_HPP_INCLUDED
