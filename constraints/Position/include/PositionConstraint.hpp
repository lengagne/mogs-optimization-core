#ifndef  PositionConstraint_HPP_
#define  PositionConstraint_HPP_

#include "MogsKinematics.h"
#include "AbstractConstraint.h"

class PositionConstraint: virtual public AbstractConstraint
{   public:
	PositionConstraint ( );

    PositionConstraint( std::vector<MogsOptimDynamics<double> *> &dyns,
                         const QString& robot_name,
                         const QString& body_name,
                         const Eigen::Matrix<double,3,1>& body_position,
                         const Eigen::Matrix<double,3,1>& desired_position);

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

    ~PositionConstraint ();

    void compute( const double*x, double * g,std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(g, dyns);
    }

    void update_dynamics(const double  *x, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        // no param here
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
