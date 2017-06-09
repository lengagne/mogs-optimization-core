#ifndef  BoxContactConstraint_HPP_
#define  BoxContactConstraint_HPP_

#include "AbstractConstraint.hpp"
#include "BoxCollisionConstraint.hpp"

class BoxContactConstraint: virtual public AbstractConstraint, virtual public BoxCollisionConstraint
{   public:
	BoxContactConstraint (QDomElement Constraint,
                          std::vector<MogsOptimDynamics<double> *>& dyns);

    ~BoxContactConstraint ();

    void compute(const double* x, double * g, std::vector<MogsOptimDynamics<double> *>& dyns);

    template<typename T>
    void update_dynamics(const  T *x, std::vector<MogsOptimDynamics<T> *>& dyns);

    template<typename T>
    void compute_contact_constraint( const T*x , T *g, std::vector<MogsOptimDynamics<T> *>& dyns);

    protected:
        unsigned int nb_contact_;

        unsigned int offset_distance_point_, offset_force_;

};

#include "BoxContactConstraint.hxx"

#endif
