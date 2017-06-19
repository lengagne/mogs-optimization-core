#ifndef BalanceFAD_1_4Constraint_HPP_
#define BalanceFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "BalanceConstraint.hpp"

class BalanceFAD_1_4Constraint: public AbstractFAD_1_4Constraint, BalanceConstraint
{
 public:
	BalanceFAD_1_4Constraint ();

    ~BalanceFAD_1_4Constraint();

    void compute( const Number *x ,Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        BalanceConstraint::compute<Number>(x,g, dyns);
    }

    void compute(  const F<Number>* x ,F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        BalanceConstraint::compute<F<Number> >(x,g, dyns);
    }

    void compute(  const Dependency *x ,Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        BalanceConstraint::compute<Dependency>(x,g, dyns);
    }

    void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns)
    {
        BalanceConstraint::update_dynamics<Number>(x, dyns);
    }

    void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns)
    {
        BalanceConstraint::update_dynamics<F<Number>>(x, dyns);
    }

    void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns)
    {
        BalanceConstraint::update_dynamics<Dependency>(x, dyns);
    }

};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
