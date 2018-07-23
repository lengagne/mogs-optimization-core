#include <BoxCollisionDoubleConstraint.hpp>

BoxCollisionDoubleConstraint::BoxCollisionDoubleConstraint() : BoxCollisionConstraint()
{

}

void BoxCollisionDoubleConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    BoxCollisionConstraint::init_from_AbstractConstraint(c);
    distance_properties_.resize(nb_body1_*nb_body2_);
}

void BoxCollisionDoubleConstraint::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    BoxCollisionConstraint::init_from_xml(constraint,dyns);
    distance_properties_.resize(nb_body1_*nb_body2_);
}

BoxCollisionDoubleConstraint::~BoxCollisionDoubleConstraint ()
{

}

void BoxCollisionDoubleConstraint::compute(const double*x,double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    SpatialTransform<double> T1,T2;
    unsigned int cpt = 0;
    for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
    {
        dyns[coll_[cpt].robot_1]->getFrameCoordinate(coll_[cpt].body_1,T1);
        dyns[coll_[cpt].robot_2]->getFrameCoordinate(coll_[cpt].body_2,T2);
        g[offset+cpt] = coll_detector_->compute_one_distance<double>(T1,T2,d1_[i],d2_[j],NULL,NULL,&distance_properties_[cpt]);
        cpt++;
    }
    computed_in_double = true;
}

extern "C" BoxCollisionDoubleConstraint* create( )
{
    return new BoxCollisionDoubleConstraint();
}

extern "C" void destroy(BoxCollisionDoubleConstraint* p)
{
    delete p;
}
