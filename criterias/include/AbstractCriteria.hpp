#ifndef AbstractCriteria_HPP_
#define AbstractCriteria_HPP_

#include "MogsKinematics.h"

class AbstractCriteria
{
 public:

    virtual double compute( const double *x , MogsKinematics<double> * kin) = 0;


	double weight_=1;
};

#endif // AbstractCriteria_HPP_INCLUDED
