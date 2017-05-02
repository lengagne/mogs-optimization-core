#ifndef AbstractConstraint_HPP_
#define AbstractConstraint_HPP_

#include "MogsKinematics.h"
#include "ToZeroConstraint.hxx"

class AbstractConstraint
{
 public:

        virtual double compute( const double *x , MogsKinematics<double> * kin, bool* compute_kin) = 0;
        int get_nnz_jac_g();
        bool eval_g(Index n, Index m, const Number * x, Number * g);
        void particuliar_jacobian(Index * iRow, Index * jCol, Index n);
        void derivative(Number * values, Index n);
        //double weight_=1;
};

#endif // AbstractConstraint_HPP_INCLUDED
