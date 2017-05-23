#ifndef AbstractConstraint_HPP_
#define AbstractConstraint_HPP_

#include "MogsOptimDynamics.h"

class AbstractConstraint
{
 public:

        virtual void compute(double *g, std::vector<MogsOptimDynamics<double> *> & dyns) = 0;

        unsigned int get_offset()
        {
            return offset;
        }

        void set_offset(unsigned int offset)
        {
            this->offset = offset;
        }

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

 protected:

      unsigned int m; /// number of constraints
      Eigen::Matrix<double, Eigen::Dynamic, 1> upper ;
      Eigen::Matrix<double, Eigen::Dynamic, 1> lower ;
      unsigned int offset;

};

#endif // AbstractConstraint_HPP_INCLUDED
