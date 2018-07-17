#ifndef StaticPostureDoubleParameterization_HPP_
#define StaticPostureDoubleParameterization_HPP_

#include "AbstractDoubleParameterization.hpp"
#include "StaticPostureParameterization.h"

class StaticPostureDoubleParameterization: public virtual AbstractDoubleParameterization, StaticPostureParameterization
{
 public:
	StaticPostureDoubleParameterization (   );

    ~StaticPostureDoubleParameterization();

    void compute( const double *x , std::vector<MogsOptimDynamics<double>*> & dyns)
    {
        StaticPostureParameterization::compute<double>(x,dyns);
    }

    void prepare_computation( std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        StaticPostureParameterization::prepare_computation<double>(dyns);
    }

};
#endif // StaticPostureDoubleParameterization_HPP_
