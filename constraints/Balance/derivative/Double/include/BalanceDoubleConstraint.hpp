#ifndef BalanceDoubleConstraint_HPP_
#define BalanceDoubleConstraint_HPP_

#include "AbstractDoubleConstraint.hpp"
#include "BalanceConstraint.hpp"

class BalanceDoubleConstraint: public AbstractDoubleConstraint, BalanceConstraint
{
 public:
	BalanceDoubleConstraint ();

    ~BalanceDoubleConstraint();

    void compute( const double *x ,double* g,std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        BalanceConstraint::compute<double>(x,g, dyns);
    }

    void update_dynamics(const double *x, std::vector<MogsOptimDynamics<double> *> & dyns)
    {
        BalanceConstraint::update_dynamics<double>(x, dyns);
    }

};
#endif // PositionDoubleCritere_HPP_INCLUDED
