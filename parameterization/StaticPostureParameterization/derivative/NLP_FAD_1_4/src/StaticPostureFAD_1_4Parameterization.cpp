#include <StaticPostureFAD_1_4Parameterization.hpp>

StaticPostureFAD_1_4Parameterization::StaticPostureFAD_1_4Parameterization (QDomElement Param,
                                                                            std::vector<MogsOptimDynamics<double> *>& dyns ):StaticPostureParameterization( Param,dyns)
{

}

StaticPostureFAD_1_4Parameterization::~StaticPostureFAD_1_4Parameterization ()
{

}

extern "C" StaticPostureFAD_1_4Parameterization* create(QDomElement Param,
                                                        std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new StaticPostureFAD_1_4Parameterization(Param,dyns);
}

extern "C" void destroy(StaticPostureFAD_1_4Parameterization* p)
{
    delete p;
}
