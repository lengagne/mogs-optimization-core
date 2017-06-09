#include <BoxContactFAD_1_4Constraint.hpp>

BoxContactFAD_1_4Constraint::BoxContactFAD_1_4Constraint(QDomElement constraint,
                                                         std::vector<MogsOptimDynamics<double> *>& dyns):
                                                             BoxCollisionConstraint(constraint,dyns),
                                                             BoxContactConstraint(constraint,dyns),
                                                             BoxCollisionFAD_1_4Constraint(constraint,dyns)
{

}

BoxContactFAD_1_4Constraint::~BoxContactFAD_1_4Constraint ()
{

}

void BoxContactFAD_1_4Constraint::compute(const F<Number>*x , F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
{
    BoxCollisionFAD_1_4Constraint::compute(x,g,dyns);
    compute_distance_point<F<Number>>(x,g,dyns);
}

void BoxContactFAD_1_4Constraint::compute( const Dependency*x, Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns)
{
    // Here we do not care of computation validity, only the dependency matters.
    BoxCollisionFAD_1_4Constraint::compute(x,g,dyns);
    Eigen::Matrix<Dependency,3,1> point, tmp;
    unsigned int cpt = offset_distance_point_;
    SpatialTransform<Dependency> trans;
    unsigned int cpt_coll = 0;
    for (int i=0;i<nb_body1_;i++) for (int j=0;j<nb_body2_;j++)
    {
        for(int k=0;k<3;k++)
            point(k) = x[offset_param_ + 6*cpt_coll + k];
        dyns[robot1_]->getFrameCoordinate(body1_[i],trans);
        tmp = trans.get_Position(point);
        g[cpt++] = tmp(0)+tmp(1)+tmp(2);
        dyns[robot2_]->getFrameCoordinate(body2_[j],trans);
        tmp = trans.get_Position(point);
        g[cpt++] = tmp(0)+tmp(1)+tmp(2);
        cpt_coll++;
    }
}

void BoxContactFAD_1_4Constraint::compute(const Number*x, Number * g, std::vector<MogsOptimDynamics<Number> *>& dyns)
{
    BoxCollisionFAD_1_4Constraint::compute(x,g,dyns);
    compute_distance_point<Number>(x,g,dyns);
}


extern "C" BoxContactFAD_1_4Constraint* create(QDomElement constraint, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new BoxContactFAD_1_4Constraint(constraint, dyns);
}

extern "C" void destroy(BoxContactFAD_1_4Constraint* p)
{
    delete p;
}
