#ifndef  TransformationConstraint_HPP_
#define  TransformationConstraint_HPP_

#include "MogsKinematics.h"
#include "AbstractConstraint.hpp"

class TransformationConstraint: virtual public AbstractConstraint
{   public:
	TransformationConstraint ( );

    TransformationConstraint( std::vector<MogsOptimDynamics<double> *> &dyns,
                             const QString& robot_name1,
                             const QString& body_name1,
                             const SpatialTransform<double> & t1,
                             const QString& robot_name2,
                             const QString& body_name2,
                             const SpatialTransform<double> & t2,
                             const SpatialTransform<double> & t_diff);

    void init(   std::vector<MogsOptimDynamics<double> *> &dyns,
                 const QString& robot_name1,
                 const QString& body_name1,
                 const QString& robot_name2,
                 const QString& body_name2 );

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

    ~TransformationConstraint ();

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
        QString Body1, Body2;
        QString Robot1, Robot2;
        SpatialTransform<double> Transformation_;

        unsigned int robot1_id_,robot2_id_,body1_id_,body2_id_;

};

#include "TransformationConstraint.hxx"

#endif
