#ifndef ABSTRACTFAD_1_4CONSTRAINT_HPP_INCLUDED
#define ABSTRACTFAD_1_4CONSTRAINT_HPP_INCLUDED

#include "AbstractConstraint.h"
#include <fadiff.h>
#include "MogsIpoptProblem.hpp"
#include "Dependency.h"

class AbstractFAD_1_4Constraint : virtual public AbstractConstraint
{
    public:

//      From AbstractConstraint
    virtual void compute(const Number*x,Number *g, std::vector<MogsOptimDynamics<Number> *>& dyn) = 0;

    virtual void compute(const F<Number>*x,F<Number>* g, std::vector<MogsOptimDynamics<F<Number> > *>& dyn) = 0;

    virtual void compute(const Dependency*x,Dependency* g, std::vector<MogsOptimDynamics<Dependency> * >& dyn) = 0;

    virtual void update_dynamics(const Number *x, std::vector<MogsOptimDynamics<Number> *> & dyns) = 0;

    virtual void update_dynamics(const F<Number> *x, std::vector<MogsOptimDynamics<F<Number>> *> & dyns) = 0;

    virtual void update_dynamics(const Dependency *x, std::vector<MogsOptimDynamics<Dependency> *> & dyns) = 0;

};

// the types of the class factories
typedef AbstractFAD_1_4Constraint* create_FAD_1_4Constraint( );

typedef void destroy_FAD_1_4Constraint(AbstractFAD_1_4Constraint*);

#endif  // ABSTRACTFAD_1_4CONSTRAINT_HPP_INCLUDED
