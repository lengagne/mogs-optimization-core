#ifndef CameraFAD_1_4Critere_HPP_
#define CameraFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "CameraCriteria.hpp"

class CameraFAD_1_4Critere: public AbstractFAD_1_4Critere, CameraCriteria
{
 public:
	CameraFAD_1_4Critere (QDomElement critere,
                          std::vector<MogsOptimDynamics<Number> *>& dyns);

    ~CameraFAD_1_4Critere ();


    Number compute( const Number *x , std::vector<MogsOptimDynamics<Number> *>& dyns, bool* compute_kin)
    {
        return CameraCriteria::compute<Number>(x,dyns, compute_kin);
    }

    F<Number>  compute( const F<Number>  *x , std::vector<MogsOptimDynamics<F<Number>>*>& dyns, bool* compute_kin)
    {
        return CameraCriteria::compute<F<Number> >(x,dyns, compute_kin);
    }


};
#endif // CameraFAD_1_4Critere_HPP_INCLUDED
