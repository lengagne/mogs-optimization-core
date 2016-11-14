#ifndef  PositionCriteria_HPP_
#define  PositionCriteria_HPP_

#include "AbstractCriteria.hpp"
#include "MogsKinematics.h"

class PositionCriteria: public AbstractCriteria
{   public:
	PositionCriteria (QDomElement critere,
                          MogsKinematics<double>* kin);

    ~PositionCriteria ();

    double compute( const double *x , MogsKinematics<double> * kin, bool* compute_kin)
    {
        return compute<double>(x,kin, compute_kin);
    }

    template<typename T>
      T compute( const T *x,MogsKinematics<T> *kin_, bool* compute_kin);

      private:

        QString Body;
        QString Robot;
        QString desired_position;
        QString body_position;
        double tval;

        unsigned int robot_id_;
        unsigned int body_id_;

        Eigen::Matrix<double, 3, 1> body_position_;
        Eigen::Matrix<double, 3, 1> desired_position_;

};

#include "PositionCriteria.hxx"

#endif // PositionCriteria_HPP_
