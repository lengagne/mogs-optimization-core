#ifndef CameraFAD_1_4Critere_HPP_
#define CameraFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "CameraCriteria.hpp"

class CameraFAD_1_4Critere: public AbstractFAD_1_4Critere, CameraCriteria
{
 public:
	CameraFAD_1_4Critere ();

    ~CameraFAD_1_4Critere ();

    virtual void init_from_xml( QDomElement criteria,
                        std::vector<MogsOptimDynamics<double> *>& dyns );

    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

    Number compute( std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        return CameraCriteria::compute<Number>(dyns);
    }

    F<Number>  compute(std::vector<MogsOptimDynamics<F<Number>>*>& dyns)
    {
        return CameraCriteria::compute<F<Number> >(dyns);
    }


};
#endif // CameraFAD_1_4Critere_HPP_INCLUDED
