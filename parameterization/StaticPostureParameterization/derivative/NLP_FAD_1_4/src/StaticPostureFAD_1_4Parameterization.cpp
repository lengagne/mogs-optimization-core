#include <StaticPostureFAD_1_4Parameterization.hpp>

StaticPostureFAD_1_4Parameterization::StaticPostureFAD_1_4Parameterization ( ):StaticPostureParameterization( )
{

}

StaticPostureFAD_1_4Parameterization::~StaticPostureFAD_1_4Parameterization ()
{

}

extern "C" StaticPostureFAD_1_4Parameterization* create( )
{
    return new StaticPostureFAD_1_4Parameterization();
}

extern "C" void destroy(StaticPostureFAD_1_4Parameterization* p)
{
    delete p;
}
