#include <BoxCollisionConstraint.hpp>


BoxCollisionConstraint::BoxCollisionConstraint (   )
{

}

void BoxCollisionConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    *this =  *(dynamic_cast<BoxCollisionConstraint*>(c));
}

void BoxCollisionConstraint::init_from_xml( QDomElement ele,
                                            std::vector<MogsOptimDynamics<double> *>& dyns )
{

//    qDebug()<<"Constructor of BoxCollisionConstraint";
    QDomElement ElRobot1=ele.firstChildElement("robot1");
    QDomElement ElRobot2=ele.firstChildElement("robot2");

    QString r1 = ElRobot1.firstChildElement("name").text().simplified();
    QString r2 = ElRobot2.firstChildElement("name").text().simplified();
    QString config1 = ElRobot1.firstChildElement("config_file").text().simplified();
    QString config2 = ElRobot2.firstChildElement("config_file").text().simplified();

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

	std::cout<<"robot1 = "<< robot1_ <<std::endl;
	for (int i=0;i<body1_.size();i++)
		std::cout<<"body1_["<<i<<"] = "<< body1_[i] <<std::endl;
	std::cout<<"robot2 = "<< robot2_ <<std::endl;
	for (int i=0;i<body2_.size();i++)
		std::cout<<"body2_["<<i<<"] = "<< body2_[i] <<std::endl;
}

BoxCollisionConstraint::~BoxCollisionConstraint ()
{
    delete coll_detector_;
    for (int i=0;i<nb_body1_;i++)
        delete d1_[i];
    for (int i=0;i<nb_body2_;i++)
        delete d2_[i];
}

void BoxCollisionConstraint::compute(double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
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

