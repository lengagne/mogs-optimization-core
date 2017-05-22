#ifndef StaticPostureFAD_1_4Parameterization_HPP_
#define StaticPostureFAD_1_4Parameterization_HPP_

#include "AbstractFAD_1_4Parameterization.hpp"
#include "StaticPostureParameterization.h"

class StaticPostureFAD_1_4Parameterization: public AbstractFAD_1_4Parameterization, StaticPostureParameterization
{
 public:
	StaticPostureFAD_1_4Parameterization (  QDomElement Param,
                                            std::vector<MogsDynamics<double> *>& dyns);

    ~StaticPostureFAD_1_4Parameterization();

    void compute( const Number *x , std::vector<MogsDynamics<Number>*> & dyns)
    {
        StaticPostureParameterization::compute<Number>(x,dyns);
    }

    void compute( const F<Number> *x , std::vector<MogsDynamics<F<Number>>*> & dyns)
    {
        StaticPostureParameterization::compute<F<Number>>(x,dyns);
    }

    void  compute( const Dependency  *x, std::vector<MogsDynamics<Dependency> *>& dyns)
    {
        StaticPostureParameterization::compute<Dependency>(x,dyns);
    }
};
#endif // StaticPostureFAD_1_4Parameterization_HPP_
