#ifndef CloseToMiddleFAD_1_4Critere_HPP_
#define CloseToMiddleFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "AbstractFAD_1_4Critere.hpp"
#include "CloseToMiddleCriteria.hpp"

class CloseToMiddleFAD_1_4Critere: public AbstractFAD_1_4Critere, CloseToMiddleCriteria
{
 public:
	CloseToMiddleFAD_1_4Critere (QDomElement critere,
                                 std::vector<MogsOptimDynamics<double> *>& dyns);

    ~CloseToMiddleFAD_1_4Critere ();

    Number compute( std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        return CloseToMiddleCriteria::compute<Number>(dyns);
    }

    F<Number>  compute(std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        return CloseToMiddleCriteria::compute<F<Number> >(dyns);
    }


};
#endif // CloseToMiddleFAD_1_4Critere_HPP_
