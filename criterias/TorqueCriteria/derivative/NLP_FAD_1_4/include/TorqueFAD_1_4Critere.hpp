#ifndef TorqueFAD_1_4Critere_HPP_
#define TorqueFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "AbstractFAD_1_4Criteria.hpp"
#include "TorqueCriteria.hpp"

class TorqueFAD_1_4Critere: public AbstractFAD_1_4Criteria, TorqueCriteria
{
 public:
	TorqueFAD_1_4Critere ( );

    ~TorqueFAD_1_4Critere ();

    virtual void init_from_xml( QDomElement criteria,
                        std::vector<MogsOptimDynamics<double> *>& dyns );

    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

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
