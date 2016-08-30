#ifndef  POSITIONADOLCCRITERE_HPP_
#define  POSITIONADOLCCRITERE_HPP_


#include <NLP_adolc.hpp>
#include <adolc.h>
#include "AbstractAdolcCritere.hpp"
#include "MogsNlpIpopt.hpp"
#include "MogsKinematics.h"

class PositionAdolcCritere: public AbstractAdolcCritere
{   public:
	PositionAdolcCritere (QDomElement critere,
                          MogsKinematics<double>* kin);

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

        QString Body;
        QString Robot;
        QString desired_position;
        QString body_position;
        double tval;
        double weight_=1;
        unsigned int robot_id_;
        unsigned int body_id_;

        Eigen::Matrix<double, 3, 1> body_position_;
        Eigen::Matrix<double, 3, 1> desired_position_;

        //std::vector<AbstractAdolcCritere* > criteres_;
};

#include "PositionAdolcCritere.hxx"

#endif // POSITIONADOLCCRITERE_HPP_
