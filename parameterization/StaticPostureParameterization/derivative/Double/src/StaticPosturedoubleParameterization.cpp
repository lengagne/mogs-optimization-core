#include <StaticPostureDoubleParameterization.hpp>

StaticPostureDoubleParameterization::StaticPostureDoubleParameterization ( ):StaticPostureParameterization(  )
{

}

StaticPostureDoubleParameterization::~StaticPostureDoubleParameterization ()
{

}

extern "C" StaticPostureDoubleParameterization* create()
{
    return new StaticPostureDoubleParameterization( );
}

extern "C" void destroy(StaticPostureDoubleParameterization* p)
{
    delete p;
}
