#ifndef  PositionDConstraint_HPP_
#define  PositionDConstraint_HPP_

#include "MogsKinematics.h"
#include "AbstractConstraint.hpp"

class PositionDConstraint: virtual public AbstractConstraint
{   public:
	PositionDConstraint (QDomElement Constraint,
                          MogsKinematics<double>* kin);

    ~PositionDConstraint ();

    void compute( const double *x , double * g, MogsKinematics<double> * kin, bool* compute_kin)
    {
        return compute<double>(x,g, kin, compute_kin);
    }

    template<typename T>
    void compute( const T *x, T *g, MogsKinematics<T> *kin_, bool* compute_kin);


    private:
        QString Body;
        QString Robot;
        QString desired_Position;
        QString body_Position;
        double tval;

        unsigned int robot_id_;
        unsigned int body_id_;

        Eigen::Matrix<double, 3, 1> body_Position_;
        Eigen::Matrix<double, 3, 1> desired_Position_;


        int n; // number of dof
};

#include "PositionDConstraint.hxx"

#endif
