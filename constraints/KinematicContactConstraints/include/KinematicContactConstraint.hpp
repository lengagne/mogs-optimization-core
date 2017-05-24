#ifndef  KinematicContactConstraint_HPP_
#define  KinematicContactConstraint_HPP_

#include "AbstractConstraint.hpp"

class KinematicContactConstraint: virtual public AbstractConstraint
{   public:
	KinematicContactConstraint (QDomElement Constraint,
                          std::vector<MogsOptimDynamics<double> *>& dyns);

    ~KinematicContactConstraint ();

    void compute(double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(g, dyns);
    }

    template<typename T>
    void compute(T *g, std::vector<MogsOptimDynamics<T> *>& dyns);


    private:

      QString Robot;
      int n; // number of dof
};

#include "KinematicContactConstraint.hxx"

#endif
