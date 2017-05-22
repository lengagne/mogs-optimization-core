#ifndef AbstractCriteria_HPP_
#define AbstractCriteria_HPP_

#include "MogsOptimDynamics.h"

class AbstractCriteria
{
 public:
    /// FIXME voir comment mieux faire le compute_kin
    virtual double compute( const double *x , std::vector<MogsOptimDynamics<double> *>& dyns, bool* compute_kin) = 0;

	double weight_=1;

};

#endif // AbstractCriteria_HPP_INCLUDED
