#include <BoxCollisionConstraint.hpp>


BoxCollisionConstraint::BoxCollisionConstraint (   )
{
    plugin_name_ = "BoxCollision";
}

BoxCollisionConstraint::BoxCollisionConstraint(  std::vector<MogsOptimDynamics<double> *> &dyns,
                                                 const QString& robot1,
                                                 const QString& robot2,
                                                 const std::vector<QString> &body1,
                                                 const std::vector<QString> &body2):BoxCollisionConstraint()
{
    init(dyns,robot1,robot2,body1,body2);
}

void  BoxCollisionConstraint::init(  std::vector<MogsOptimDynamics<double> *> &dyns,
                                     const QString& robot1,
                                     const QString& robot2,
                                     const std::vector<QString> &body1,
                                     const std::vector<QString> &body2)
{
    unsigned int nb = dyns.size();
    for (int i=0;i<nb;i++)
    {
        if( dyns[i]->getRobotName() == robot1)
        {
            robot1_ = i;
            break;
        }
    }
    for (int i=0;i<nb;i++)
    {
        if( dyns[i]->getRobotName() == robot2)
        {
            robot2_ = i;
            break;
        }
    }

    QString config1, config2;
    if(!dyns[robot1_]->model->get_config_file("BoxCollision",config1))
    {
        std::cerr<<"Error in "<< __FILE__<<" at line "<<__LINE__<<std::endl;
        std::cerr<<"Cannot find the config file (BoxCollision) for robot1_"<<std::endl;
        exit(0);
    };
    if(!dyns[robot2_]->model->get_config_file("BoxCollision",config2))
    {
        std::cerr<<"Error in "<< __FILE__<<" at line "<<__LINE__<<std::endl;
        std::cerr<<"Cannot find the config file (BoxCollision) for robot2_"<<std::endl;
        exit(0);
    };

    for (int i = 0; i < body1.size(); i++)
    {
        QString body1_name = body1[i];
        d1_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(config1),body1_name));
    }

    nb_body1_ = body1.size();

    for (int i = 0; i < body2.size(); i++)
    {
        QString body2_name = body2[i];
        d2_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(config2),body2_name));
    }
    nb_body2_ = body2.size();

    m = nb_body1_ * nb_body2_;

    for (int i=0;i<nb_body1_;i++)
        for (int j=0;j<nb_body2_;j++)
        {
            collision_value tmp;
            tmp.robot_1 = robot1_;
            tmp.robot_2 = robot2_;
            body1_.push_back(dyns[robot1_]->model->GetBodyId(body1[i]));
            body2_.push_back(dyns[robot2_]->model->GetBodyId(body2[j]));
            tmp.body_1 = dyns[robot1_]->model->GetBodyId(body1[i]);
            tmp.body_2 = dyns[robot2_]->model->GetBodyId(body2[j]);
            coll_.push_back(tmp);
        }

    upper_.resize(m);
    lower_.resize(m);
    for (int i=0;i<m;i++)
    {
        upper_[i] = lower_[i] =0;

    }
    coll_detector_ = new MogsBoxCollision();

}

void BoxCollisionConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    *this =  *(dynamic_cast<BoxCollisionConstraint*>(c));
}

void BoxCollisionConstraint::init_from_xml( QDomElement ele,
                                            std::vector<MogsOptimDynamics<double> *>& dyns )
{
    QDomElement ElRobot1=ele.firstChildElement("robot1");
    QDomElement ElRobot2=ele.firstChildElement("robot2");

    QString r1 = ElRobot1.firstChildElement("name").text().simplified();
    QString r2 = ElRobot2.firstChildElement("name").text().simplified();

    unsigned int nb = dyns.size();
    for (int i=0;i<nb;i++)
    {
        if( dyns[i]->getRobotName() == r1)
        {
            robot1_ = i;
            break;
        }
    }
    for (int i=0;i<nb;i++)
    {
        if( dyns[i]->getRobotName() == r2)
        {
            robot2_ = i;
            break;
        }
    }

    QString config1, config2;
//    config1 = dyns[robot1_]->model->get_config_file("BoxCollision");
//    config2 = dyns[robot2_]->model->get_config_file("BoxCollision");
    if(!dyns[robot1_]->model->get_config_file("BoxCollision",config1))
    {
        std::cerr<<"Error in "<< __FILE__<<" at line "<<__LINE__<<std::endl;
        std::cerr<<"Cannot find the config file (BoxCollision) for robot1_"<<std::endl;
        exit(0);
    };
    if(!dyns[robot2_]->model->get_config_file("BoxCollision",config2))
    {
        std::cerr<<"Error in "<< __FILE__<<" at line "<<__LINE__<<std::endl;
        std::cerr<<"Cannot find the config file (BoxCollision) for robot2_"<<std::endl;
        exit(0);
    };


//    QString config1 = ElRobot1.firstChildElement("config_file").text().simplified();
//    QString config2 = ElRobot2.firstChildElement("config_file").text().simplified();

    if(ElRobot1.isNull())
    {
        qDebug()<<"Error in constraint BoxCollisionConstraint tag \"robot1\" not found";
        exit(1);
    }
    if(ElRobot2.isNull())
    {
        qDebug()<<"Error in constraint BoxCollisionConstraint tag \"robot2\" not found";
        exit(1);
    }

    std::vector<QString> b1;
    for (QDomElement ElBody1 = ele.firstChildElement("body1"); !ElBody1.isNull(); ElBody1 = ElBody1.nextSiblingElement("body1") )
    {
        QString body1_name = ElBody1.text().simplified();
        b1.push_back(body1_name);
        d1_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(config1),body1_name));
    }

    nb_body1_ = b1.size();

    std::vector<QString> b2;
    for (QDomElement ElBody2 = ele.firstChildElement("body2"); !ElBody2.isNull(); ElBody2 = ElBody2.nextSiblingElement("body2"))
    {
        QString body2_name = ElBody2.text().simplified();
        b2.push_back(body2_name);
        d2_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(config2),body2_name));
    }
    nb_body2_ = b2.size();

    m = nb_body1_ * nb_body2_;

    for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
    {
        collision_value tmp;
        tmp.robot_1 = robot1_;
        tmp.robot_2 = robot2_;
        body1_.push_back(dyns[robot1_]->model->GetBodyId(b1[i]));
        body2_.push_back(dyns[robot2_]->model->GetBodyId(b2[j]));
        tmp.body_1 = dyns[robot1_]->model->GetBodyId(b1[i]);
        tmp.body_2 = dyns[robot2_]->model->GetBodyId(b2[j]);
        coll_.push_back(tmp);
    }

    QDomElement Eltype=ele.firstChildElement("type");
    upper_.resize(m);
    lower_.resize(m);
    for (int i=0;i<m;i++)
    {
        upper_[i] = lower_[i] =0;
        if (!Eltype.isNull())
        {
            if (Eltype.text().simplified()=="zero")
            {

            }else if(Eltype.text().simplified()=="avoid")
            {
                lower_[i] = 0.001;
                upper_[i] = 1e10;
            }
            else if(Eltype.text().simplified()=="penetration")
                lower_[i] = -1e10;
        }
    }
    coll_detector_ = new MogsBoxCollision();
}

BoxCollisionConstraint::~BoxCollisionConstraint ()
{
    delete coll_detector_;
    for (int i=0;i<nb_body1_;i++)
        delete d1_[i];
    for (int i=0;i<nb_body2_;i++)
        delete d2_[i];
}

void BoxCollisionConstraint::compute( const double *x ,double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    SpatialTransform<double> T1,T2;
    unsigned int cpt = 0;
    for (int i=0;i<nb_body1_;i++)   for (int j=0;j<nb_body2_;j++)
    {
        dyns[robot1_]->getFrameCoordinate(body1_[i],T1);
        dyns[robot2_]->getFrameCoordinate(body2_[j],T2);

        g[offset+cpt] = coll_detector_->compute_one_distance<double>(T1,T2,d1_[i],d2_[j]);
        cpt++;
    }
}

