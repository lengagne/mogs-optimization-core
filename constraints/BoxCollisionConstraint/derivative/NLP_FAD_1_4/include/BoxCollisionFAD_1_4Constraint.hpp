#ifndef BoxCollisionFAD_1_4Constraint_HPP_
#define BoxCollisionFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "BoxCollisionConstraint.hpp"

class BoxCollisionFAD_1_4Constraint: virtual public AbstractFAD_1_4Constraint, virtual public BoxCollisionConstraint
{
 public:
	BoxCollisionFAD_1_4Constraint (QDomElement constraint,
                                std::vector<MogsOptimDynamics<double> *>& dyns);

    ~BoxCollisionFAD_1_4Constraint();

    inline void compute_double(const double *x, double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        compute(x,g,dyns);
    }

    void compute(const Number*x, Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns);

    void compute(const F<Number>*x, F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns);

    void compute(const Dependency*x, Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns);

    void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns)
    {
        BoxCollisionConstraint::update_dynamics<Number>(x, dyns);
    }

    void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns)
    {
        BoxCollisionConstraint::update_dynamics<F<Number>>(x, dyns);
    }

    void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns)
    {
        BoxCollisionConstraint::update_dynamics<Dependency>(x, dyns);
    }


};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
