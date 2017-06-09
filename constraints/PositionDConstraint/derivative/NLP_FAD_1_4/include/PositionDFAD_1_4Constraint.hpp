#ifndef PositionDFAD_1_4Constraint_HPP_
#define PositionDFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "PositionDConstraint.hpp"

class PositionDFAD_1_4Constraint: public AbstractFAD_1_4Constraint, PositionDConstraint
{
 public:
	PositionDFAD_1_4Constraint (QDomElement constraint,
                                std::vector<MogsOptimDynamics<double> *>& dyns);

    ~PositionDFAD_1_4Constraint();

    void compute( const Number*x, Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        PositionDConstraint::compute<Number>(g, dyns);
    }

    void compute(const F<Number>*x,F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        PositionDConstraint::compute<F<Number> >(g, dyns);
    }

    void compute( const Dependency*x,Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        PositionDConstraint::compute<Dependency>(g, dyns);
    }

    void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns)
    {
        PositionDConstraint::update_dynamics<Number>(x, dyns);
    }

    void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns)
    {
        PositionDConstraint::update_dynamics<F<Number>>(x, dyns);
    }

    void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns)
    {
        PositionDConstraint::update_dynamics<Dependency>(x, dyns);
    }
};
#endif // PositionDFAD_1_4Critere_HPP_INCLUDED
