#include <BoxCollisionFAD_1_4Constraint.hpp>

BoxCollisionFAD_1_4Constraint::BoxCollisionFAD_1_4Constraint (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns):BoxCollisionConstraint(constraint,dyns)
{

}

BoxCollisionFAD_1_4Constraint::~BoxCollisionFAD_1_4Constraint ()
{

}

void BoxCollisionFAD_1_4Constraint::compute( const F<Number>* x, F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
{
   SpatialTransform<F<Number> > T1,T2;
    unsigned int cpt = 0;
    for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
    {
        dyns[coll_[cpt].robot_1]->getFrameCoordinate(coll_[cpt].body_1,T1);
        dyns[coll_[cpt].robot_2]->getFrameCoordinate(coll_[cpt].body_2,T2);


        g[offset+cpt] = coll_detector_->compute_one_distance<F<Number>>(T1,T2,d1_[i],d2_[j]);
        cpt++;
    }
//    dyns[coll_.robot_1]->getFrameCoordinate(coll_.body_1,T1);
//    dyns[coll_.robot_2]->getFrameCoordinate(coll_.body_2,T2);
//    g[offset] =  coll_detector_->compute_one_distance<F<Number>>(T1,T2,coll_,d1_,d2_);
}

void BoxCollisionFAD_1_4Constraint::compute( const Dependency* x, Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
{
    unsigned int cpt = 0;
    for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
    {
        Eigen::Matrix<Dependency,3,1> P1 =     dyns[coll_[cpt].robot_1]->getPosition(coll_[cpt].body_1,coll_[cpt].point_1);
        Eigen::Matrix<Dependency,3,1> P2 =     dyns[coll_[cpt].robot_2]->getPosition(coll_[cpt].body_2,coll_[cpt].point_2);
        g[offset+cpt] = P1(0)+P1(1)+P1(2)+P2(0)+P2(1)+P2(2);
        cpt ++;
    }

}

void BoxCollisionFAD_1_4Constraint::compute(const Number*x,Number * g, std::vector<MogsOptimDynamics<Number> *>& dyns)
{
    SpatialTransform<Number> T1,T2;
    unsigned int cpt = 0;
    for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
    {
        dyns[coll_[cpt].robot_1]->getFrameCoordinate(coll_[cpt].body_1,T1);
        dyns[coll_[cpt].robot_2]->getFrameCoordinate(coll_[cpt].body_2,T2);
        g[offset+cpt] = coll_detector_->compute_one_distance<Number>(T1,T2,d1_[i],d2_[j]);
        cpt++;
    }
}


extern "C" BoxCollisionFAD_1_4Constraint* create(QDomElement constraint, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new BoxCollisionFAD_1_4Constraint(constraint, dyns);
}

extern "C" void destroy(BoxCollisionFAD_1_4Constraint* p)
{
    delete p;
}
