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
                           MogsKinematics<Number> *kin);

    ~CameraFADBAD_1_4Critere ();

    Number compute( const Number *x , MogsKinematics<Number> * kin, bool* compute_kin)
    {
        return CameraCriteria::compute<Number>(x,kin, compute_kin);
    }

    F<Number>  compute( const F<Number>  *x , MogsKinematics<F<Number> > * kin, bool* compute_kin)
    {
        return CameraCriteria::compute<F<Number> >(x,kin, compute_kin);
    }


};
#endif // CameraFADBAD_1_4Critere_HPP_INCLUDED
