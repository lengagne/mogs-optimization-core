#include <PositionDoubleConstraint.hpp>

PositionDoubleConstraint::PositionDoubleConstraint () : PositionConstraint()
{

}

PositionDoubleConstraint::~PositionDoubleConstraint ()
{

}

void PositionDoubleConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    PositionConstraint::init_from_AbstractConstraint(c);
}

void PositionDoubleConstraint::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    PositionConstraint::init_from_xml(constraint,dyns);

}


extern "C" PositionDoubleConstraint* create( )
{
    return new PositionDoubleConstraint( );
}

extern "C" void destroy(PositionDoubleConstraint* p)
{
    delete p;
}
