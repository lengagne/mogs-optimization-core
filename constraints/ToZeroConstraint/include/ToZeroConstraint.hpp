#ifndef  ToZeroConstraint_HPP_
#define  ToZeroConstraint_HPP_

#include "MogsKinematics.h"
#include "AbstractConstraint.hpp"
#include "ToZeroConstraint.hxx"

class ToZeroConstraint: public AbstractConstraint
{   public:
	ToZeroConstraint (QDomElement Constraint,
                          MogsKinematics<double>* kin);

    ~ToZeroConstraint ();

    double compute( const double *x , MogsKinematics<double> * kin, bool* compute_kin)
    {
        return compute<double>(x,kin, compute_kin);
    }

    template<typename T>
      T compute( const T *x,MogsKinematics<T> *kin_, bool* compute_kin);


    private:

      int m;
      Eigen::Matrix<double,Eigen::Dynamic,  1> upper;
      Eigen::Matrix<double,Eigen::Dynamic,  1> lower;
      QString Robot;
      int get_nb_constraints();
      double get_upper(int i);
      double get_lower(int i);
};

#endif


// CloseToMiddleCriteria_HPP_

 //bool NLP_FAD_1_4::eval_g (Index n, const Number * x, bool new_x, Index m, Number * g)


  //    int nnz_jac_g;
    //    int nnz_jac_g;
