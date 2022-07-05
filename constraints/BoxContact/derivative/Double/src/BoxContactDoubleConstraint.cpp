#include <BoxContactDoubleConstraint.hpp>

BoxContactDoubleConstraint::BoxContactDoubleConstraint(): BoxContactConstraint()
{

}


BoxContactDoubleConstraint::~BoxContactDoubleConstraint ()
{

}

void BoxContactDoubleConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    BoxContactConstraint::init_from_AbstractConstraint(c);
    distance_properties_.resize(nb_body1_*nb_body2_);
}

void BoxContactDoubleConstraint::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    BoxContactConstraint::init_from_xml(constraint,dyns);
    distance_properties_.resize(nb_body1_*nb_body2_);
}


void BoxContactDoubleConstraint::compute(const double*x, double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    BoxCollisionDoubleConstraint::compute(x,g,dyns);
    compute_contact_constraint<double>(x,g,dyns);
}


extern "C" BoxContactDoubleConstraint* create( )
{
    return new BoxContactDoubleConstraint( );
}

extern "C" void destroy(BoxContactDoubleConstraint* p)
{
    delete p;
}
