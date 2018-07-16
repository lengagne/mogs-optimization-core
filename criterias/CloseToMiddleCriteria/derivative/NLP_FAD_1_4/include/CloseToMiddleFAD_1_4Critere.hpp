#ifndef CloseToMiddleFAD_1_4Critere_HPP_
#define CloseToMiddleFAD_1_4Critere_HPP_

#include <NLP_FAD_1_4.hpp>
#include "AbstractFAD_1_4Criteria.hpp"
#include "CloseToMiddleCriteria.hpp"

class CloseToMiddleFAD_1_4Critere: public AbstractFAD_1_4Criteria, virtual CloseToMiddleCriteria
{
 public:
	CloseToMiddleFAD_1_4Critere ( );

    ~CloseToMiddleFAD_1_4Critere ();


    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );


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
