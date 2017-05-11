#ifndef PositionFAD_1_4Critere_HPP_
#define PositionFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "AbstractFAD_1_4Critere.hpp"
#include "PositionCriteria.hpp"

class PositionFAD_1_4Critere: public AbstractFAD_1_4Critere, PositionCriteria
{
 public:
	PositionFAD_1_4Critere (QDomElement critere,
                             std::vector<MogsDynamics<Number> *> dyns);

    ~PositionFAD_1_4Critere ();

    Number compute( const Number *x ,  std::vector<MogsDynamics<Number> *> dyns, bool* compute_kin)
    {
        return PositionCriteria::compute<Number>(x,dyns, compute_kin);
    }

    F<Number>  compute( const F<Number>  *x ,  std::vector<MogsDynamics<F<Number>> *> dyns, bool* compute_kin)
    {
        return PositionCriteria::compute<F<Number> >(x,dyns, compute_kin);
    }


};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
