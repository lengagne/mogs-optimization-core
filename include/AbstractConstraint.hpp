#ifndef AbstractConstraint_HPP_
#define AbstractConstraint_HPP_

#include "MogsKinematics.h"

class AbstractConstraint
{
 public:

        virtual void compute( const double *x , double *g, MogsKinematics<double> * kin, bool* compute_kin) = 0;
//        int get_nnz_jac_g();
//        bool eval_g(Index n, Index m, const Number * x, Number * g);
//        void particuliar_jacobian(Index * iRow, Index * jCol, Index n);
//        void derivative(Number * values, Index n);
        //double weight_=1;

        unsigned int get_nb_constraints()
        {
            return m;
        }

        double get_upper(unsigned int i)
        {
            return upper(i);
        }

        double get_lower(unsigned int i)
        {
            return lower(i);
        }


 protected:
        unsigned int m; /// number of constraints
      Eigen::Matrix<double,Eigen::Dynamic,  1> upper;
      Eigen::Matrix<double,Eigen::Dynamic,  1> lower;
};

#endif // AbstractConstraint_HPP_INCLUDED
