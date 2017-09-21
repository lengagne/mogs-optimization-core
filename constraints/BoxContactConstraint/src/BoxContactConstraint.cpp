#include <BoxContactConstraint.hpp>

BoxContactConstraint::BoxContactConstraint (  ):BoxCollisionConstraint()
{
    plugin_name_ = "BoxContact";
}

BoxContactConstraint::BoxContactConstraint (std::vector<MogsOptimDynamics<double> *> &dyns,
                     const QString& robot1,
                     const QString& robot2,
                     const std::vector<QString> &body1,
                     const std::vector<QString> &body2):BoxContactConstraint()
{
    BoxCollisionConstraint::init(dyns,robot1,robot2,body1,body2);
//     qDebug()<< "robot1 : " << robot1;
//     qDebug()<<"robot2 : " << robot2;
//     qDebug()<< "body1 : " << body1[0];
//     qDebug()<<"body2 : " << body2[0];

//    unsigned int nb = dyns.size();
//    for (int i=0;i<nb;i++)
//    {
//        if( dyns[i]->getRobotName() == robot1)
//        {
//            robot1_ = i;
//            break;
//        }
//    }
//    for (int i=0;i<nb;i++)
//    {
//        if( dyns[i]->getRobotName() == robot2)
//        {
//            robot2_ = i;
//            break;
//        }
//    }
//
//    QString config1, config2;
//    config1 = dyns[robot1_]->model->get_config_file("BoxCollision");
//    config2 = dyns[robot2_]->model->get_config_file("BoxCollision");
//
////     qDebug()<<"config1 : " << config1;
////     qDebug()<<"config2 : " << config2;
//
//
////    qDebug()<<"Constructor of BoxCollisionConstraint";
//    for (int i = 0; i < body1.size(); i++)
//    {
//        QString body1_name = body1[i];
//        d1_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(config1),body1_name));
//    }
//
//    nb_body1_ = body1.size();
//
//    for (int i = 0; i < body2.size(); i++)
//    {
//        QString body2_name = body2[i];
//        d2_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(config2),body2_name));
//    }
//    nb_body2_ = body2.size();
//
//    m = nb_body1_ * nb_body2_;
//
//    for (int i=0;i<nb_body1_;i++)
//        for (int j=0;j<nb_body2_;j++)
//        {
//            collision_value tmp;
//            tmp.robot_1 = robot1_;
//            tmp.robot_2 = robot2_;
//
//            body1_.push_back(dyns[robot1_]->model->GetBodyId(body1[i]));
//            body2_.push_back(dyns[robot2_]->model->GetBodyId(body2[j]));
//            tmp.body_1 = dyns[robot1_]->model->GetBodyId(body1[i]);
//            tmp.body_2 = dyns[robot2_]->model->GetBodyId(body2[j]);
//            coll_.push_back(tmp);
//        }
//
//    upper_.resize(m);
//    lower_.resize(m);
//    for (int i=0;i<m;i++)
//    {
//        upper_[i] = lower_[i] =0;
//
//    }
//    coll_detector_ = new MogsBoxCollision();

    nb_contact_ = coll_.size();

    friction_ = 0.5;

    nb_param_ = nb_contact_ * 6;
//    std::cout<<"BoxContactConstraint::nb_param_ =  "<< nb_param_ <<std::endl;
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
        lower_.push_back(friction_); // friction cone for the moment
        upper_.push_back(1.0);
    }

}

void BoxContactConstraint::init_from_xml(   QDomElement ele,
                                            std::vector<MogsOptimDynamics<double> *>& dyns)
{
    BoxCollisionConstraint::init_from_xml(ele,dyns);
//    std::cout<<"We deal with : "<< coll_.size()<<" contacts."<<std::endl;
//    std::cout<<"BoxCollisionConstraint::m =  "<< m <<std::endl;
    nb_contact_ = coll_.size();

    friction_ = 0.5;
    QDomElement ElFriction = ele.firstChildElement("friction");
    if(!ElFriction.isNull())
    {
        friction_ = ele.text().toDouble();
        friction_ = 1/ (1+friction_*friction_);
    }

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
        lower_.push_back(friction_); // friction cone for the moment
        upper_.push_back(1.0);
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

void BoxContactConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    *this =  *(dynamic_cast<BoxContactConstraint*>(c));
}


void BoxContactConstraint::compute(const double *x , double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    BoxCollisionConstraint::compute(x,g,dyns);

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
}
#endif // MogsVisu_FOUND

