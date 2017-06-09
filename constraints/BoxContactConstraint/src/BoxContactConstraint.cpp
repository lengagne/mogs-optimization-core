#include <BoxContactConstraint.hpp>

BoxContactConstraint::BoxContactConstraint (  QDomElement ele,
                                                std::vector<MogsOptimDynamics<double> *>& dyns):
                                                    BoxCollisionConstraint(ele,dyns)

{
    std::cout<<"We deal with : "<< coll_.size()<<" contacts."<<std::endl;
    nb_contact_ = coll_.size();

    nb_param_ = nb_contact_ * 6;

    offset_distance_point_ = m;
    offset_force_ = m +2*nb_contact_;
    m += 2*nb_contact_;    // two constraints for the point and one constraint for the effort

//    coll_detector_ = new MogsBoxCollision();
    for (int i=0;i<nb_param_;i++)
    {
        param_inf_.push_back(-1e3);
        param_sup_.push_back(1e3);
        param_init_.push_back(0.1);
    }

    for (int i=0;i<nb_contact_;i++)
    {
        upper_.push_back(0.1);    lower_.push_back(-0.1);  // distance for first body
        upper_.push_back(0.1);    lower_.push_back(-0.1);  // distance for second body
        lower_.push_back(0.5); // friction cone for the moment
        upper_.push_back(1e200);
    }
}

BoxContactConstraint::~BoxContactConstraint ()
{
    // the same than BoxCollisionConstraint
    delete coll_detector_;
    for (int i=0;i<nb_body1_;i++)
        delete d1_[i];
    for (int i=0;i<nb_body2_;i++)
        delete d2_[i];
}

void BoxContactConstraint::compute(const double *x , double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    BoxCollisionConstraint::compute(g,dyns);

    compute_contact_constraint<double>(x,g,dyns);

//    SpatialTransform<double> T1,T2;
//    unsigned int cpt = 0;
//    for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
//    {
//        dyns[coll_[cpt].robot_1]->getFrameCoordinate(coll_[cpt].body_1,T1);
//        dyns[coll_[cpt].robot_2]->getFrameCoordinate(coll_[cpt].body_2,T2);
//
//
//        g[offset+cpt] = coll_detector_->compute_one_distance<double>(T1,T2,coll_[cpt],d1_[i],d2_[j]);
//        std::cout<<"g["<<offset+cpt<<"] = "<< g[offset+cpt] <<std::endl;
//        cpt++;
//    }
}

