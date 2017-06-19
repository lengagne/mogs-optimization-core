#include <BalanceFAD_1_4Constraint.hpp>

BalanceFAD_1_4Constraint::BalanceFAD_1_4Constraint ():BalanceConstraint( )
{

}

BalanceFAD_1_4Constraint::~BalanceFAD_1_4Constraint ()
{

}

extern "C" BalanceFAD_1_4Constraint* create()
{
    return new BalanceFAD_1_4Constraint( );
}

extern "C" void destroy(BalanceFAD_1_4Constraint* p)
{
    delete p;
}
