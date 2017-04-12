#ifndef  CloseToMiddleCriteria_HPP_
#define  CloseToMiddleCriteria_HPP_

#include "AbstractCriteria.hpp"
#include "MogsKinematics.h"
//#include "close_to_middleCriteria.hxx"
//#include "MogsRobotProperties.h"

class CloseToMiddleCriteria: public AbstractCriteria
{   public:
	CloseToMiddleCriteria (QDomElement critere,
                          MogsKinematics<double>* kin);

    ~CloseToMiddleCriteria ();

    double compute( const double *x , MogsKinematics<double> * kin, bool* compute_kin)
    {
        return compute<double>(x,kin, compute_kin);
    }

    template<typename T>
      T compute( const T *x,MogsKinematics<T> *kin_, bool* compute_kin);
      private:

        QString Robot;
       Eigen::Matrix<double,Eigen::Dynamic,  1> qm_;
};



#endif // PositionCriteria_HPP_
