#include <BoxContactConstraint.hpp>

BoxContactConstraint::BoxContactConstraint (  QDomElement ele,
                                                std::vector<MogsOptimDynamics<double> *>& dyns):
                                                    BoxCollisionConstraint(ele,dyns)

{
    std::cout<<"We deal with : "<< coll_.size()<<" contacts."<<std::endl;
    std::cout<<"m =  "<< m <<std::endl;
    nb_contact_ = coll_.size();

    nb_param_ = nb_contact_ * 6;
    m += 3*nb_contact_;    // two constraints for the point and one constraint for the effort

//    coll_detector_ = new MogsBoxCollision();
    for (int i=0;i<nb_param_;i++)
    {
        param_inf_.push_back(-1e3);
        param_sup_.push_back(1e3);
        param_init_.push_back(0.0);
    }


    for (int i=0;i<nb_contact_;i++)
    {
        upper_.push_back(0.);    lower_.push_back(-0.);  // distance for first body
        upper_.push_back(0.);    lower_.push_back(-0.);  // distance for second body
        lower_.push_back(0.8); // friction cone for the moment
        upper_.push_back(1.0);
    }

//    std::cout<<"robot1 = "<< robot1_<<std::endl;
//    std::cout<<"robot2 = "<< robot2_<<std::endl;
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
}

#ifdef MogsVisu_FOUND
void BoxContactConstraint::update_visu (VisuHolder *visu,
                                        std::vector<MogsOptimDynamics<double> *> & dyns,
                                        const double * x)
{

    Eigen::Matrix<double,6,1> line;
    Eigen::Matrix<double,3,1> point, cp1,cp2,normal;
    SpatialTransform<double> trans1,trans2;
    std::vector<Eigen::Matrix<double,6,1>> lines;
    lines.clear();
    unsigned int cpt_coll  = 0;
    for (int i=0;i<nb_body1_;i++) for (int j=0;j<nb_body2_;j++)
    {
        for(int k=0;k<3;k++)
        {
            line(k) = x[offset_param_ + 6*cpt_coll + k];
            line(3+k) = line(k) + x[offset_param_ + cpt_coll*6 + k+3];
        }
//         std::cout<<"point = "<< line.block(0,0,3,1).transpose()<<std::endl;
//         std::cout<<"force = "<< (line.block(3,0,3,1)-line.block(0,0,3,1)).transpose()<<std::endl;
        lines.push_back(line);
		cpt_coll++;
    }
    visu->draw_additional_lines(lines,0,0,255);

    lines.clear();
    for (int i=0;i<nb_body1_;i++) for (int j=0;j<nb_body2_;j++)
    {
        for(int k=0;k<3;k++)
            point(k) = x[offset_param_ + 6*cpt_coll + k];

        dyns[robot1_]->getFrameCoordinate(body1_[i],trans1);
        coll_detector_->compute_one_distance(trans1,d1_[i],point,cp1);
        cp1 = trans1.get_Position(cp1);
        dyns[robot2_]->getFrameCoordinate(body2_[j],trans2);
        coll_detector_->compute_normal(trans1,trans2,d1_[i],d2_[j],normal);

        for(int k=0;k<3;k++)
        {
            line(k) = cp1(k);
            line(3+k) = cp1(k)+normal(k);
        }
        lines.push_back(line);

        visu->draw_additional_lines(lines,255,0,0);
        cpt_coll++;
    }
//    for (int i=0;i<dyns.size();i++)
//        std::cout<<"dyns["<<i<<"]->q = "<< dyns[i]->q_.transpose()<<std::endl;
}
#endif // MogsVisu_FOUND

