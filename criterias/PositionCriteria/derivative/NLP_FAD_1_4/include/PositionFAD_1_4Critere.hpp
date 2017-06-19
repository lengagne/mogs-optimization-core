#ifndef PositionFAD_1_4Critere_HPP_
#define PositionFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "AbstractFAD_1_4Critere.hpp"
#include "PositionCriteria.hpp"

class PositionFAD_1_4Critere: public AbstractFAD_1_4Critere, PositionCriteria
{
 public:
	PositionFAD_1_4Critere ( );

    ~PositionFAD_1_4Critere ();

    virtual void init_from_xml( QDomElement criteria,
                        std::vector<MogsOptimDynamics<double> *>& dyns );

    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

    Number compute( std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        return PositionCriteria::compute<Number>(dyns);
    }

    F<Number>  compute( std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        return PositionCriteria::compute<F<Number> >(dyns);
    }


};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
