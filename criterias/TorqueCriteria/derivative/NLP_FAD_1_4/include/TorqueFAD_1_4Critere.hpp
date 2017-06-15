#ifndef TorqueFAD_1_4Critere_HPP_
#define TorqueFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "AbstractFAD_1_4Critere.hpp"
#include "TorqueCriteria.hpp"

class TorqueFAD_1_4Critere: public AbstractFAD_1_4Critere, TorqueCriteria
{
 public:
	TorqueFAD_1_4Critere (QDomElement critere,
                                 std::vector<MogsOptimDynamics<double> *>& dyns);

    ~TorqueFAD_1_4Critere ();

    Number compute( std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        return TorqueCriteria::compute<Number>(dyns);
    }

    F<Number>  compute(std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        return TorqueCriteria::compute<F<Number> >(dyns);
    }


};
#endif // TorqueFAD_1_4Critere_HPP_
