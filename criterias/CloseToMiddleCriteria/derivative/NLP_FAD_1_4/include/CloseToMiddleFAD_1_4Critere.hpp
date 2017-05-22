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

    Number compute( const Number *x , std::vector<MogsOptimDynamics<Number> *>& dyns, bool* compute_kin)
    {
        return CloseToMiddleCriteria::compute<Number>(x,dyns, compute_kin);
    }

    F<Number>  compute( const F<Number>  *x , std::vector<MogsOptimDynamics<F<Number>> *>& dyns, bool* compute_kin)
    {
        return CloseToMiddleCriteria::compute<F<Number> >(x,dyns, compute_kin);
    }


};
#endif // CloseToMiddleFAD_1_4Critere_HPP_
