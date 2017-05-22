#ifndef  ToZeroConstraint_HPP_
#define  ToZeroConstraint_HPP_

#include "AbstractConstraint.hpp"

class ToZeroConstraint: virtual public AbstractConstraint
{   public:
	ToZeroConstraint (QDomElement Constraint,
                          std::vector<MogsOptimDynamics<double> *>& dyns);

    ~ToZeroConstraint ();

    void compute( const double *x , double * g, std::vector<MogsOptimDynamics<double> *>& dyns, bool* compute_kin)
    {
        return compute<double>(x,g, dyns, compute_kin);
    }

    template<typename T>
    void compute( const T *x, T *g, std::vector<MogsOptimDynamics<T> *>& dyns, bool* compute_kin);


    private:

      QString Robot;
      int n; // number of dof
};

#include "ToZeroConstraint.hxx"

#endif
