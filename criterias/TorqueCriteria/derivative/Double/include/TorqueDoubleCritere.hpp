#ifndef TorqueDoubleCritere_HPP_
#define TorqueDoubleCritere_HPP_

#include "AbstractDoubleCriteria.hpp"
#include "TorqueCriteria.hpp"

class TorqueDoubleCritere: public AbstractDoubleCriteria, TorqueCriteria
{
 public:
	TorqueDoubleCritere ( );

	~TorqueDoubleCritere ();

	virtual void init_from_xml( 	QDomElement criteria,
					std::vector<MogsOptimDynamics<double> *>& dyns );

	virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

	double compute( std::vector<MogsOptimDynamics<double> *>& dyns)
	{
		return TorqueCriteria::compute<double>(dyns);
	}

};
#endif // TorqueDoubleCritere_HPP_
