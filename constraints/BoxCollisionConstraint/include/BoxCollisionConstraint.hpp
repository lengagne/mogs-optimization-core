#ifndef  BoxCollisionConstraint_HPP_
#define  BoxCollisionConstraint_HPP_

#include "AbstractConstraint.hpp"
#include "MogsBoxCollision.h"

class BoxCollisionConstraint: virtual public AbstractConstraint
{   public:
	BoxCollisionConstraint (QDomElement Constraint,
                          std::vector<MogsOptimDynamics<double> *>& dyns);

    ~BoxCollisionConstraint ();

    void compute(double * g, std::vector<MogsOptimDynamics<double> *>& dyns);

    template<typename T>
    void update_dynamics(const  T *x, std::vector<MogsOptimDynamics<T> *>& dyns)
    {

    }

    protected:

      unsigned int robot1_, robot2_;
      std::vector<unsigned int> body1_, body2_;

      unsigned int nb_body1_, nb_body2_;

      std::vector<MogsBoxCollisionDefinition*> d1_, d2_;

      MogsBoxCollision* coll_detector_;

    protected:
      std::vector<collision_value> coll_;

};

#endif
