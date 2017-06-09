#ifndef StaticPostureFAD_1_4Parameterization_HPP_
#define StaticPostureFAD_1_4Parameterization_HPP_

#include "AbstractFAD_1_4Parameterization.hpp"
#include "StaticPostureParameterization.h"

class StaticPostureFAD_1_4Parameterization: public AbstractFAD_1_4Parameterization, StaticPostureParameterization
{
 public:
	StaticPostureFAD_1_4Parameterization (  QDomElement Param,
                                            std::vector<MogsOptimDynamics<double> *>& dyns);

    ~StaticPostureFAD_1_4Parameterization();

    void compute( const Number *x , std::vector<MogsOptimDynamics<Number>*> & dyns)
    {
        StaticPostureParameterization::compute<Number>(x,dyns);
    }

    void compute( const F<Number> *x , std::vector<MogsOptimDynamics<F<Number>>*> & dyns)
    {
        StaticPostureParameterization::compute<F<Number>>(x,dyns);
    }

    void  compute( const Dependency  *x, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        StaticPostureParameterization::compute<Dependency>(x,dyns);
    }

    void prepare_computation( std::vector<MogsOptimDynamics<Number> *>& dyns)
    {
        StaticPostureParameterization::prepare_computation<Number>(dyns);
    }

    void prepare_computation( std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
    {
        StaticPostureParameterization::prepare_computation<F<Number>>(dyns);
    }

    void prepare_computation( std::vector<MogsOptimDynamics<Dependency> *>& dyns)
    {
        StaticPostureParameterization::prepare_computation<Dependency>(dyns);
    }
};
#endif // StaticPostureFAD_1_4Parameterization_HPP_
