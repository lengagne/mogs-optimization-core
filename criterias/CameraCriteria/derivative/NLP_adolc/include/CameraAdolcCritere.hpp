#ifndef CameraAdolcCritere_HPP_
#define CameraAdolcCritere_HPP_

#include <adolc.h>
#include "AbstractAdolcCritere.hpp"
#include "CameraCriteria.hpp"

class CameraAdolcCritere: public AbstractAdolcCritere, CameraCriteria
{
 public:
	CameraAdolcCritere (QDomElement critere,
                           MogsKinematics<double> *kin);

    ~CameraAdolcCritere ();

    double compute( const double *x , MogsKinematics<double> * kin, bool* compute_kin)
    {
        return CameraCriteria::compute<double>(x,kin, compute_kin);
    }

    adouble compute( const adouble *x , MogsKinematics<adouble> * kin, bool* compute_kin)
    {
        return CameraCriteria::compute<adouble>(x,kin, compute_kin);
    }


};
#endif // CameraAdolcCritere_HPP_INCLUDED
