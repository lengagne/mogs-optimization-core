#ifndef BoxContactFAD_1_4Constraint_HPP_
#define BoxContactFAD_1_4Constraint_HPP_

#include "BoxCollisionFAD_1_4Constraint.hpp"
#include "BoxContactConstraint.hpp"

class BoxContactFAD_1_4Constraint: public BoxCollisionFAD_1_4Constraint, public BoxContactConstraint
{
 public:
	BoxContactFAD_1_4Constraint (QDomElement constraint,
                                std::vector<MogsOptimDynamics<double> *>& dyns);

    ~BoxContactFAD_1_4Constraint();

    void compute(const Number*x, Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns);

    void compute(const F<Number>*x,  F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns);

    void compute(const Dependency*x,  Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns);

    void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns)
    {
        BoxContactConstraint::update_dynamics<Number>(x, dyns);
    }

    void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns)
    {
        BoxContactConstraint::update_dynamics<F<Number>>(x, dyns);
    }

    void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns)
    {
        BoxContactConstraint::update_dynamics<Dependency>(x, dyns);
    }

};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
