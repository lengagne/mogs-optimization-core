#ifndef CameraAdolcCritere_HPP_
#define CameraAdolcCritere_HPP_

#include <NLP_adolc.hpp>
#include <adolc.h>
#include "AbstractAdolcCritere.hpp"
#include "CameraCriteria.hpp"
#include "MogsNlpIpopt.hpp"

class CameraAdolcCritere: public AbstractAdolcCritere, CameraCriteria
{
 public:
	CameraAdolcCritere (QDomElement critere,
                           MogsKinematics<double> *kin);

    ~CameraAdolcCritere ();

    double compute( const double *x , MogsKinematics<double> * kin)
    {
        return CameraCriteria::compute<double>(x,kin);
    }

    adouble compute( const adouble *x , MogsKinematics<adouble> * kin)
    {
        return CameraCriteria::compute<adouble>(x,kin);
    }


};
#endif // CameraAdolcCritere_HPP_INCLUDED
