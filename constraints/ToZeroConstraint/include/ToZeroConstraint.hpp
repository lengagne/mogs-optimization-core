#ifndef  ToZeroConstraint_HPP_
#define  ToZeroConstraint_HPP_

#include "AbstractConstraint.hpp"

class ToZeroConstraint: virtual public AbstractConstraint
{   public:
	ToZeroConstraint (QDomElement Constraint,
                          std::vector<MogsOptimDynamics<double> *>& dyns);

    ~ToZeroConstraint ();

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

#include "ToZeroConstraint.hxx"

#endif
