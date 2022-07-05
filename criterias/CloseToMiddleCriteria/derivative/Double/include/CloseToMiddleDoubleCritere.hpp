#ifndef CloseToMiddleDoubleCritere_HPP_
#define CloseToMiddleDoubleCritere_HPP_

#include "AbstractDoubleCriteria.hpp"
#include "CloseToMiddleCriteria.hpp"

class CloseToMiddleDoubleCritere: public AbstractDoubleCriteria, virtual CloseToMiddleCriteria
{
 public:
	CloseToMiddleDoubleCritere ( );

    ~CloseToMiddleDoubleCritere ();

    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );


    double compute( std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return CloseToMiddleCriteria::compute<double>(dyns);
    }
};
#endif // CloseToMiddleDoubleCritere_HPP_
