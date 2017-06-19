#include <PositionFAD_1_4Constraint.hpp>

PositionFAD_1_4Constraint::PositionFAD_1_4Constraint () : PositionConstraint()
{

}

PositionFAD_1_4Constraint::~PositionFAD_1_4Constraint ()
{

}

void PositionFAD_1_4Constraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    PositionConstraint::init_from_AbstractConstraint(c);
}

void PositionFAD_1_4Constraint::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    PositionConstraint::init_from_xml(constraint,dyns);

}


extern "C" PositionFAD_1_4Constraint* create( )
{
    return new PositionFAD_1_4Constraint( );
}

extern "C" void destroy(PositionFAD_1_4Constraint* p)
{
    delete p;
}
