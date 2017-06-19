#ifndef  PositionConstraint_HPP_
#define  PositionConstraint_HPP_

#include "MogsKinematics.h"
#include "AbstractConstraint.hpp"

class PositionConstraint: virtual public AbstractConstraint
{   public:
	PositionConstraint (QDomElement Constraint,
                         std::vector<MogsOptimDynamics<double> *>& dyns);

    ~PositionConstraint ();

    void compute( const double*x, double * g,std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(g, dyns);
    }

    template<typename T>
    void compute( T *g, std::vector<MogsOptimDynamics<T> *>& dyns);

    template<typename T>
    void update_dynamics(const  T *x, std::vector<MogsOptimDynamics<T> *>& dyns)
    {
        // no param here
    }

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

};

#include "PositionConstraint.hxx"

#endif
