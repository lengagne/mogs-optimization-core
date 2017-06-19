#include <BoxCollisionFAD_1_4Constraint.hpp>

BoxCollisionFAD_1_4Constraint::BoxCollisionFAD_1_4Constraint() : BoxCollisionConstraint()
{

}

void BoxCollisionFAD_1_4Constraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    BoxCollisionConstraint::init_from_AbstractConstraint(c);
    distance_properties_.resize(nb_body1_*nb_body2_);
}

void BoxCollisionFAD_1_4Constraint::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    BoxCollisionConstraint::init_from_xml(constraint,dyns);
    distance_properties_.resize(nb_body1_*nb_body2_);
}

BoxCollisionFAD_1_4Constraint::~BoxCollisionFAD_1_4Constraint ()
{

}

void BoxCollisionFAD_1_4Constraint::compute( const F<Number>* x, F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns)
{
    SpatialTransform<F<Number> > T1,T2;
    unsigned int cpt = 0;
    if(computed_in_Number)
    {
        for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
        {
            dyns[coll_[cpt].robot_1]->getFrameCoordinate(coll_[cpt].body_1,T1);
            dyns[coll_[cpt].robot_2]->getFrameCoordinate(coll_[cpt].body_2,T2);
            g[offset+cpt] = coll_detector_->compute_one_distance_property<F<Number>>(T1,T2,d1_[i],d2_[j],distance_properties_[cpt]);
            cpt++;
        }
    }else
    {
        SpatialTransform<F<Number> > T1,T2;
        SpatialTransform<Number> NT1,NT2;
        unsigned int cpt = 0;
        for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
        {
            dyns[coll_[cpt].robot_1]->getFrameCoordinate(coll_[cpt].body_1,T1);
            dyns[coll_[cpt].robot_2]->getFrameCoordinate(coll_[cpt].body_2,T2);

            for (int ii=0;ii<3;ii++)
            {
                NT1.r(ii) = T1.r(ii).x();
                NT2.r(ii) = T2.r(ii).x();
                for (int jj=0;jj<3;jj++)
                {
                    NT1.E(ii,jj) = T1.E(ii,jj).x();
                    NT2.E(ii,jj) = T2.E(ii,jj).x();
                }
            }
            coll_detector_->compute_one_distance<Number>(NT1,NT2,d1_[i],d2_[j],NULL,NULL,&distance_properties_[cpt]);
            g[offset+cpt] = coll_detector_->compute_one_distance_property<F<Number>>(T1,T2,d1_[i],d2_[j],distance_properties_[cpt]);

            //g[offset+cpt] = coll_detector_->compute_one_distance<F<Number>>(T1,T2,d1_[i],d2_[j]);

            cpt++;
        }
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
        g[offset+cpt] = coll_detector_->compute_one_distance<Number>(T1,T2,d1_[i],d2_[j],NULL,NULL,&distance_properties_[cpt]);
        cpt++;
    }
    computed_in_Number = true;
}


extern "C" BoxCollisionFAD_1_4Constraint* create( )
{
    return new BoxCollisionFAD_1_4Constraint();
}

extern "C" void destroy(BoxCollisionFAD_1_4Constraint* p)
{
    delete p;
}
