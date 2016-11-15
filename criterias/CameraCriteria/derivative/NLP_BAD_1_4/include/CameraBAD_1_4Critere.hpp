#ifndef CameraBAD_1_4Critere_HPP_
#define CameraBAD_1_4Critere_HPP_

#include <NLP_BAD_1_4.hpp>
#include "CameraCriteria.hpp"

class CameraBAD_1_4Critere: public AbstractBAD_1_4Critere, CameraCriteria
{
 public:
	CameraBAD_1_4Critere (QDomElement critere,
                           MogsKinematics<Number> *kin);

    ~CameraBAD_1_4Critere ();

    Number compute( const Number *x , MogsKinematics<Number> * kin, bool* compute_kin)
    {
        return CameraCriteria::compute<Number>(x,kin, compute_kin);
    }

    B<Number>  compute( const B<Number>  *x , MogsKinematics<B<Number> > * kin, bool* compute_kin)
    {
        return CameraCriteria::compute<B<Number> >(x,kin, compute_kin);
    }


};
#endif // CameraBAD_1_4Critere_HPP_INCLUDED
