#include <BalanceDoubleConstraint.hpp>

BalanceDoubleConstraint::BalanceDoubleConstraint ():BalanceConstraint( )
{

}

BalanceDoubleConstraint::~BalanceDoubleConstraint ()
{

}

extern "C" BalanceDoubleConstraint* create()
{
    return new BalanceDoubleConstraint( );
}

extern "C" void destroy(BalanceDoubleConstraint* p)
{
    delete p;
}
