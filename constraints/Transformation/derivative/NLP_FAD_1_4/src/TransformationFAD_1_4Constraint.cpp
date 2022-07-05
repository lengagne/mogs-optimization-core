#include <TransformationFAD_1_4Constraint.hpp>

TransformationFAD_1_4Constraint::TransformationFAD_1_4Constraint () : TransformationConstraint()
{

}

TransformationFAD_1_4Constraint::~TransformationFAD_1_4Constraint ()
{

}

void TransformationFAD_1_4Constraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    TransformationConstraint::init_from_AbstractConstraint(c);
}

void TransformationFAD_1_4Constraint::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    TransformationConstraint::init_from_xml(constraint,dyns);

}


extern "C" TransformationFAD_1_4Constraint* create( )
{
    return new TransformationFAD_1_4Constraint( );
}

extern "C" void destroy(TransformationFAD_1_4Constraint* p)
{
    delete p;
}
