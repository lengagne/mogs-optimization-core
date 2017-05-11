#ifndef  ToZeroConstraint_HPP_
#define  ToZeroConstraint_HPP_

#include "MogsKinematics.h"
#include "AbstractConstraint.hpp"

class ToZeroConstraint: virtual public AbstractConstraint
{   public:
	ToZeroConstraint (QDomElement Constraint,
                          MogsKinematics<double>* kin);

    ~ToZeroConstraint ();

    void compute( const double *x , double * g, MogsKinematics<double> * kin, bool* compute_kin)
    {
        return compute<double>(x,g, kin, compute_kin);
    }

    template<typename T>
    void compute( const T *x, T *g, MogsKinematics<T> *kin_, bool* compute_kin);


    private:

      QString Robot;
      int n; // number of dof
};

#include "ToZeroConstraint.hxx"

#endif
