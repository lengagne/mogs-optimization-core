#ifndef TransformationFAD_1_4Constraint_HPP_
#define TransformationFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "TransformationConstraint.hpp"

class TransformationFAD_1_4Constraint: public AbstractFAD_1_4Constraint, TransformationConstraint
{
 public:
	TransformationFAD_1_4Constraint ( );

    ~TransformationFAD_1_4Constraint();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

    void compute( const Number*x, Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        TransformationConstraint::compute<Number>(g, dyns);
    }

    void compute(const F<Number>*x,F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        TransformationConstraint::compute<F<Number> >(g, dyns);
    }

    void compute( const Dependency*x,Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        TransformationConstraint::compute<Dependency>(g, dyns);
    }

    void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns)
    {
        TransformationConstraint::update_dynamics<Number>(x, dyns);
    }

    void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns)
    {
        TransformationConstraint::update_dynamics<F<Number>>(x, dyns);
    }

    void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns)
    {
        TransformationConstraint::update_dynamics<Dependency>(x, dyns);
    }
};
#endif // TransformationFAD_1_4Critere_HPP_INCLUDED
