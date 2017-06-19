#ifndef PositionFAD_1_4Constraint_HPP_
#define PositionFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "PositionConstraint.hpp"

class PositionFAD_1_4Constraint: public AbstractFAD_1_4Constraint, PositionConstraint
{
 public:
	PositionFAD_1_4Constraint (QDomElement constraint,
                                std::vector<MogsOptimDynamics<double> *>& dyns);

    ~PositionFAD_1_4Constraint();

    void compute( const Number*x, Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        PositionConstraint::compute<Number>(g, dyns);
    }

    void compute(const F<Number>*x,F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        PositionConstraint::compute<F<Number> >(g, dyns);
    }

    void compute( const Dependency*x,Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        PositionConstraint::compute<Dependency>(g, dyns);
    }

    void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns)
    {
        PositionConstraint::update_dynamics<Number>(x, dyns);
    }

    void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns)
    {
        PositionConstraint::update_dynamics<F<Number>>(x, dyns);
    }

    void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns)
    {
        PositionConstraint::update_dynamics<Dependency>(x, dyns);
    }
};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
