#ifndef  POSITIONADOLCCRITERE_HPP_
#define  POSITIONADOLCCRITERE_HPP_



#include <adolc.h>
#include "MogsRobotProperties.h"
#include "AbstractAdolcCritere.hpp"

class PositionAdolcCritere: public AbstractAdolcCritere
{
      public:


	PositionAdolcCritere ();
    ~PositionAdolcCritere ();



    double compute( const double *x , MogsKinematics<double> * kin)
    {
        return compute<double>(x,kin);
    }

    adouble compute( const adouble *x , MogsKinematics<adouble> * kin)
    {
        return compute<adouble>(x,kin);
    }

 template<typename T>
      T compute( const T *x,MogsKinematics<T> *kin_);

      private:
        QString body_;
        QString robot_;

        unsigned int robot_id_;
        unsigned int body_id_;

        Eigen::Matrix<double, 3, 1> body_position_;
        Eigen::Matrix<double, 3, 1> desired_position_;
};

#include "PositionAdolcCritere.hxx"

#endif // POSITIONADOLCCRITERE_HPP_
