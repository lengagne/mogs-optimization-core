#ifndef  PositionDConstraint_HPP_
#define  PositionDConstraint_HPP_

#include "MogsKinematics.h"
#include "AbstractConstraint.hpp"

class PositionDConstraint: virtual public AbstractConstraint
{   public:
	PositionDConstraint (QDomElement Constraint,
                         std::vector<MogsOptimDynamics<double> *>& dyns);

    ~PositionDConstraint ();

    void compute( double * g,std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(g, dyns);
    }

    template<typename T>
    void compute( T *g, std::vector<MogsOptimDynamics<T> *>& dyns);


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


//        int n; // number of dof
};

#include "PositionDConstraint.hxx"

#endif
