#include <TorqueDoubleConstraint.hpp>

TorqueDoubleConstraint::TorqueDoubleConstraint () : TorqueConstraint()
{

}

TorqueDoubleConstraint::~TorqueDoubleConstraint ()
{

}

void TorqueDoubleConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    TorqueConstraint::init_from_AbstractConstraint(c);
}

void TorqueDoubleConstraint::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    TorqueConstraint::init_from_xml(constraint,dyns);

}


extern "C" TorqueDoubleConstraint* create( )
{
    return new TorqueDoubleConstraint( );
}

extern "C" void destroy(TorqueDoubleConstraint* p)
{
    delete p;
}
