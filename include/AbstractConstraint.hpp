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

        double get_upper(int i)
        {
            return upper(i);
        }

        double get_lower( int i)
        {
            return lower(i);
        }

        int get_test( )
        {
            return test;
        }

 protected:

      unsigned int m; /// number of constraints
      Eigen::Matrix<double, 3, 1> upper ;
      Eigen::Matrix<double, 3, 1> lower ;

      int test;

};

#endif // AbstractConstraint_HPP_INCLUDED
