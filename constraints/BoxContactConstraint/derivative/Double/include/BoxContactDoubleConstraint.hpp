#ifndef BoxContactDoubleConstraint_HPP_
#define BoxContactDoubleConstraint_HPP_

#include "BoxCollisionDoubleConstraint.hpp"
#include "BoxContactConstraint.hpp"

class BoxContactDoubleConstraint: public virtual BoxCollisionDoubleConstraint, public BoxContactConstraint
{
 public:
	BoxContactDoubleConstraint ( );

    ~BoxContactDoubleConstraint();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

    void compute(const double*x, double* g,std::vector<MogsOptimDynamics<double> *>& dyns);

    void update_dynamics(const double *x, std::vector<MogsOptimDynamics<double> *> & dyns)
    {
        BoxContactConstraint::update_dynamics<double>(x, dyns);
    }

};
#endif // PositionDoubleCritere_HPP_INCLUDED
