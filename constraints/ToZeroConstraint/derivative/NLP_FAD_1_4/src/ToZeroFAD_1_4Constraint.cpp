#include <ToZeroFAD_1_4Constraint.hpp>

ToZeroFAD_1_4Constraint::ToZeroFAD_1_4Constraint ( ):ToZeroConstraint()
{

}

ToZeroFAD_1_4Constraint::~ToZeroFAD_1_4Constraint ()
{

}

extern "C" ToZeroFAD_1_4Constraint* create()
{
    return new ToZeroFAD_1_4Constraint( );
}

extern "C" void destroy(ToZeroFAD_1_4Constraint* p)
{
    delete p;
}
