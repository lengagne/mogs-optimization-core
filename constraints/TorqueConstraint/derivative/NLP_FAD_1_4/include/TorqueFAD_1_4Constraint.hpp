#ifndef TorqueFAD_1_4Constraint_HPP_
#define TorqueFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "TorqueConstraint.hpp"

class TorqueFAD_1_4Constraint: public AbstractFAD_1_4Constraint, TorqueConstraint
{
 public:
	TorqueFAD_1_4Constraint ();

    ~TorqueFAD_1_4Constraint();

    void compute( const Number *x ,Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        TorqueConstraint::compute<Number>(x,g, dyns);
    }

    void compute(  const F<Number>* x ,F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        TorqueConstraint::compute<F<Number> >(x,g, dyns);
    }

    void compute(  const Dependency *x ,Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        TorqueConstraint::compute<Dependency>(x,g, dyns);
    }

    void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns)
    {
        TorqueConstraint::update_dynamics<Number>(x, dyns);
    }

    void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns)
    {
        TorqueConstraint::update_dynamics<F<Number>>(x, dyns);
    }

    void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns)
    {
        TorqueConstraint::update_dynamics<Dependency>(x, dyns);
    }

};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
