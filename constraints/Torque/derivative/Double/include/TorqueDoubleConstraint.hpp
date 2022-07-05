#ifndef TorqueDoubleConstraint_HPP_
#define TorqueDoubleConstraint_HPP_

#include "AbstractDoubleConstraint.hpp"
#include "TorqueConstraint.hpp"

class TorqueDoubleConstraint: public AbstractDoubleConstraint, TorqueConstraint
{
 public:
	TorqueDoubleConstraint ( );

    ~TorqueDoubleConstraint();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

    void compute( const double*x, double* g,std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        TorqueConstraint::compute<double>(g, dyns);
    }

    void update_dynamics(const double *x, std::vector<MogsOptimDynamics<double> *> & dyns)
    {
        TorqueConstraint::update_dynamics<double>(x, dyns);
    }
};
#endif // TorqueDoubleCritere_HPP_INCLUDED
