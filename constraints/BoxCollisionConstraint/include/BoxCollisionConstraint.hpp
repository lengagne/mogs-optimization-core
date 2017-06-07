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

    protected:

      unsigned int robot1_, robot2_, body1_, body2_;

      MogsBoxCollisionDefinition *d1_, *d2_;

      MogsBoxCollision* coll_detector_;

    protected:
      collision_value coll_;

    std::vector<MogsKinematics<double> *> dyns_;

};

#endif
