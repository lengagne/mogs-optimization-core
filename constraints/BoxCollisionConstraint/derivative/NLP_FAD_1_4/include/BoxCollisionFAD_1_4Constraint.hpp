#ifndef BoxCollisionFAD_1_4Constraint_HPP_
#define BoxCollisionFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "BoxCollisionConstraint.hpp"

class BoxCollisionFAD_1_4Constraint: virtual public AbstractFAD_1_4Constraint, virtual public BoxCollisionConstraint
{
 public:
	BoxCollisionFAD_1_4Constraint ( );

    ~BoxCollisionFAD_1_4Constraint();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );


    inline void compute_double(const double *x, double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        compute(x,g,dyns);
    }

    void compute(const Number*x, Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns);

    void compute(const F<Number>*x, F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns);

    void compute(const Dependency*x, Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns);

    void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns)
    {
        computed_in_Number = false;
        BoxCollisionConstraint::update_dynamics<Number>(x, dyns);
    }

    void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns)
    {
        BoxCollisionConstraint::update_dynamics<F<Number>>(x, dyns);
    }

    void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns)
    {
        computed_in_Number = false;
        BoxCollisionConstraint::update_dynamics<Dependency>(x, dyns);
    }

    bool computed_in_Number;
    std::vector<DistanceProperties> distance_properties_;
};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
