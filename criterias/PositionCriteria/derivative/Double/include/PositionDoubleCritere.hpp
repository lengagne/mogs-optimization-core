#ifndef PositionDoubleCritere_HPP_
#define PositionDoubleCritere_HPP_

#include "AbstractDoubleCriteria.hpp"
#include "PositionCriteria.hpp"

class PositionDoubleCritere: public AbstractDoubleCriteria, PositionCriteria
{
 public:
	PositionDoubleCritere ( );

    ~PositionDoubleCritere ();

    virtual void init_from_xml( QDomElement criteria,
                        std::vector<MogsOptimDynamics<double> *>& dyns );

    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

    double compute( std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return PositionCriteria::compute<double>(dyns);
    }

};
#endif // PositionDoubleCritere_HPP_INCLUDED
