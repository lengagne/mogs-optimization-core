#ifndef PositionDoubleConstraint_HPP_
#define PositionDoubleConstraint_HPP_

#include "AbstractDoubleConstraint.hpp"
#include "PositionConstraint.hpp"

class PositionDoubleConstraint: public AbstractDoubleConstraint, PositionConstraint
{
 public:
	PositionDoubleConstraint ( );

    ~PositionDoubleConstraint();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

    void compute( const double*x, double* g,std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        PositionConstraint::compute<double>(g, dyns);
    }

    void update_dynamics(const double *x, std::vector<MogsOptimDynamics<double> *> & dyns)
    {
        PositionConstraint::update_dynamics<double>(x, dyns);
    }
};
#endif // PositionDoubleCritere_HPP_INCLUDED
