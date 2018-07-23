#include <TorqueFAD_1_4Constraint.hpp>

TorqueFAD_1_4Constraint::TorqueFAD_1_4Constraint ():TorqueConstraint( )
{

}

TorqueFAD_1_4Constraint::~TorqueFAD_1_4Constraint ()
{

}

extern "C" TorqueFAD_1_4Constraint* create()
{
    return new TorqueFAD_1_4Constraint( );
}

extern "C" void destroy(TorqueFAD_1_4Constraint* p)
{
    delete p;
}
