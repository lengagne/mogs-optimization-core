#ifndef  BoxCollisionConstraint_HPP_
#define  BoxCollisionConstraint_HPP_

#include "MogsKinematics.h"
#include "AbstractConstraint.hpp"
#include "MogsBoxCollision.h"

class BoxCollisionConstraint: virtual public AbstractConstraint
{   public:
	BoxCollisionConstraint ( );
    BoxCollisionConstraint(  std::vector<MogsOptimDynamics<double> *> &dyns,
                             const QString& robot1_,
                             const QString& robot2_,
                             const QString& body1_,
                             const QString& body2_,
                             const Eigen::Matrix<double,3,1>& body_position);


    ~BoxCollisionConstraint ();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

    void compute( const double *x ,double * g, std::vector<MogsOptimDynamics<double> *>& dyns);


    void update_dynamics(const double *x, std::vector<MogsOptimDynamics<double> *> & dyns)
    {
        update_dynamics<double>(x,dyns);
    }

    template<typename T>
    void update_dynamics(const  T *x, std::vector<MogsOptimDynamics<T> *>& dyns)
    {

    }

    protected:

      unsigned int robot1_, robot2_;
      std::vector<unsigned int> body1_, body2_;

      unsigned int nb_body1_, nb_body2_;

      std::vector<MogsBoxCollisionDefinition*> d1_, d2_;

      MogsBoxCollision* coll_detector_;

    protected:
      std::vector<collision_value> coll_;

};

#endif
