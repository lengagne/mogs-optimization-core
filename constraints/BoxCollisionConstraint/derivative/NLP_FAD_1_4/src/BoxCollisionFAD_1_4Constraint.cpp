#include <BoxCollisionFAD_1_4Constraint.hpp>

BoxCollisionFAD_1_4Constraint::BoxCollisionFAD_1_4Constraint (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns):BoxCollisionConstraint(constraint,dyns)
{

}

BoxCollisionFAD_1_4Constraint::~BoxCollisionFAD_1_4Constraint ()
{

}

void BoxCollisionFAD_1_4Constraint::compute( F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
{
    SpatialTransform<F<Number> > T1,T2;
    dyns[coll_.robot_1]->getFrameCoordinate(coll_.body_1,T1);
    dyns[coll_.robot_2]->getFrameCoordinate(coll_.body_2,T2);
    g[offset] =  coll_detector_->compute_one_distance<F<Number>>(T1,T2,coll_,d1_,d2_);
}

void BoxCollisionFAD_1_4Constraint::compute( Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
{
    Eigen::Matrix<Dependency,3,1> P1 =     dyns[coll_.robot_1]->getPosition(coll_.body_1,coll_.point_1);
    Eigen::Matrix<Dependency,3,1> P2 =     dyns[coll_.robot_1]->getPosition(coll_.body_2,coll_.point_2);
    g[offset] = P1(0)+P1(1)+P1(2)+P2(0)+P2(1)+P2(2);
}

void BoxCollisionFAD_1_4Constraint::compute(Number * g, std::vector<MogsOptimDynamics<Number> *>& dyns)
{
    SpatialTransform<Number> T1,T2;
    dyns[coll_.robot_1]->getFrameCoordinate(coll_.body_1,T1);
    dyns[coll_.robot_2]->getFrameCoordinate(coll_.body_2,T2);
    g[offset] =  coll_detector_->compute_one_distance<Number>(T1,T2,coll_,d1_,d2_);
//    std::cout<<"g["<<offset<<"] = "<< g[offset]  <<std::endl;
}


extern "C" BoxCollisionFAD_1_4Constraint* create(QDomElement constraint, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new BoxCollisionFAD_1_4Constraint(constraint, dyns);
}

extern "C" void destroy(BoxCollisionFAD_1_4Constraint* p)
{
    delete p;
}
