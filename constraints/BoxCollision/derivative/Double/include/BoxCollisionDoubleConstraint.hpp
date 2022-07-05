#ifndef BoxCollisionDoubleConstraint_HPP_
#define BoxCollisionDoubleConstraint_HPP_

#include "AbstractDoubleConstraint.hpp"
#include "BoxCollisionConstraint.hpp"

class BoxCollisionDoubleConstraint: virtual public AbstractDoubleConstraint, virtual public BoxCollisionConstraint
{
 public:
	BoxCollisionDoubleConstraint ( );

    ~BoxCollisionDoubleConstraint();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );


    inline void compute_double(const double *x, double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        compute(x,g,dyns);
    }

    void compute(const double*x, double* g,std::vector<MogsOptimDynamics<double> *>& dyns);

    void update_dynamics(const double *x, std::vector<MogsOptimDynamics<double> *> & dyns)
    {
        computed_in_double = false;
        BoxCollisionConstraint::update_dynamics<double>(x, dyns);
    }

    bool computed_in_double;
    std::vector<DistanceProperties> distance_properties_;
};
#endif // PositionDoubleCritere_HPP_INCLUDED
