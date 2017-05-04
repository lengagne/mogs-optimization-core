#ifndef AbstractConstraint_HPP_
#define AbstractConstraint_HPP_

#include "MogsKinematics.h"

class AbstractConstraint
{
 public:

        virtual void compute( const double *x , double *g, MogsKinematics<double> * kin, bool* compute_kin) = 0;

        unsigned int get_nb_constraints()
        {
            return m;
        }

        double get_upper()
        {
            return upper;
        }

        double get_lower()
        {
            return lower;
        }

 protected:
      unsigned int m; /// number of constraints

      double upper = 0;
      double lower = 0;
};

#endif // AbstractConstraint_HPP_INCLUDED
